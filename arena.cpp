// arena.cpp
// Filtro de partículas do centro do dojô no referencial do robô (explicação no arena.h).
// Cada partícula k é uma hipótese (px[k], py[k]) de onde fica o centro do dojô visto do robô,
// em mm, com peso pw[k]. O filtro roda no núcleo 1, dentro do percepcao_atualiza.

#include "arena.h"
#include "config.h"
#include <math.h>

static const float R_DOJO = DOHYO_DIAMETRO_MM / 2.0f;
static const float R_FAIXA = R_DOJO - BORDA_FAIXA_MM;     // começo da faixa branca
static const float LARGO = 50.0f;                         // largura da cauda do modelo do branco (mm)

static float    px[ARENA_N], py[ARENA_N], pw[ARENA_N];
// Escala do movimento de cada partícula. Sem encoder, o modelo das rodas erra de forma
// sistemática (bateria, atrito, pneu), e um erro sorteado de novo a cada passo se cancela e
// esconde esse desvio. Cada partícula carrega então a sua escala de avanço e de giro, e as
// escalas que explicam a borda são as que sobrevivem na reamostragem.
static float    sv[ARENA_N], sg[ARENA_N];
// Em contato o modelo das rodas não vale: o robô pode estar travado, patinando, sendo empurrado
// ou sendo girado. Cada partícula sorteia, no começo do contato, quanto do avanço (ca) e do giro
// (cg) pedidos aconteceu de verdade e uma velocidade de empurrão (ex, ey, em mm/s) que dura o
// contato inteiro. Um sorteio novo a cada passo se cancelaria em média (todas as partículas
// andariam mais ou menos a metade do pedido); guardado, ele cobre de verdade o robô parado ou
// arrastado, e a borda escolhe as hipóteses certas na reamostragem.
static float    ca[ARENA_N], cg[ARENA_N], ex[ARENA_N], ey[ARENA_N];
static bool     contatoAnt = false;
static float    qx[ARENA_N], qy[ARENA_N], qv[ARENA_N], qg[ARENA_N];   // rascunho da reamostragem
static float    qca[ARENA_N], qcg[ARENA_N], qex[ARENA_N], qey[ARENA_N];
static bool     valida = false;
static uint32_t tAnt = 0, tPasso = 0;
static float    accDx = 0, accDth = 0, accDt = 0;           // movimento estimado desde o último passo
static bool     accContato = false;
static uint16_t accBranco = 0, accPreto = 0xFFFF;           // borda vista entre dois passos
static uint16_t ultBranco = 0;                              // sensores no branco no passo anterior
static uint32_t semente = 0x9E3779B9u;
static uint32_t usPasso = 0, usPior = 0;                   // tempo do passo do filtro (comando arena)
static uint32_t tSurpresa = 0;                             // última vez em que o branco contrariou o filtro
#ifndef ARENA_SURPRESA
#define ARENA_SURPRESA 0.15f
#endif
#ifndef ARENA_T_SURPRESA_MS
#define ARENA_T_SURPRESA_MS 300
#endif
static Arena    saida;

// A saída é lida também no núcleo 0 (descarte das leituras de fora do dojô), então a cópia
// passa por uma trava curta.
#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
static portMUX_TYPE trava = portMUX_INITIALIZER_UNLOCKED;
#define TRAVA()    portENTER_CRITICAL(&trava)
#define DESTRAVA() portEXIT_CRITICAL(&trava)
#else
#define TRAVA()
#define DESTRAVA()
#endif

// gerador xorshift32: rápido e suficiente para o ruído das partículas
static inline float aleat() {
  semente ^= semente << 13; semente ^= semente >> 17; semente ^= semente << 5;
  return (semente >> 8) * (1.0f / 16777216.0f);
}
// normal aproximada (soma de 4 uniformes, desvio 1)
static inline float gauss() { return (aleat() + aleat() + aleat() + aleat() - 2.0f) * 1.7320508f; }
// triangular (soma de 2 uniformes, desvio 1), para o ruído de cada passo da predição
static inline float tri() { return (aleat() + aleat() - 1.0f) * 2.4494897f; }

// Sorteio de uma partícula compatível com o começo da luta: robô inteiro na sua metade e com
// os 4 sensores de borda no preto.
static bool sorteiaInicio(float& x, float& y, bool opVisto, float ox, float oy) {
  for (int tent = 0; tent < 60; tent++) {
    float d = ARENA_D_MIN + aleat() * (ARENA_D_MAX - ARENA_D_MIN);
    float a = (aleat() * 2.0f - 1.0f) * (float)PI;
    x = d * cosf(a); y = d * sinf(a);
    bool ok = true;
    for (uint8_t j = 0; j < N_BORDA && ok; j++)
      if (hypotf(BORDA[j].x - x, BORDA[j].y - y) > R_FAIXA - 10) ok = false;
    // Regra da CoRA: os dois robôs começam em metades opostas, atrás das linhas Shikiri. O
    // centro então fica entre os dois: projetado na linha até o oponente, a pelo menos uns 12 cm
    // de cada robô. A folga de 6 cm cobre o erro da estimativa do oponente.
    if (ok && opVisto) {
      float od = hypotf(ox, oy);
      float proj = (x * ox + y * oy) / (od > 1 ? od : 1);
      if (proj < 60 || proj > od - 60) ok = aleat() < 0.25f;   // a estimativa do oponente pode estar errada
    }
    if (ok) return true;
  }
  return false;
}

static void calculaSaida() {
  float sw = 0, mx = 0, my = 0;
  for (int k = 0; k < ARENA_N; k++) { sw += pw[k]; mx += pw[k] * px[k]; my += pw[k] * py[k]; }
  if (sw <= 0) sw = 1;
  mx /= sw; my /= sw;
  float sxx = 0, syy = 0, sxy = 0;
  for (int k = 0; k < ARENA_N; k++) {
    float dx = px[k] - mx, dy = py[k] - my;
    sxx += pw[k] * dx * dx; syy += pw[k] * dy * dy; sxy += pw[k] * dx * dy;
  }
  sxx /= sw; syy /= sw; sxy /= sw;
  // maior autovalor da covariância 2x2
  float tr = sxx + syy, det = sxx * syy - sxy * sxy;
  float disc = tr * tr * 0.25f - det;
  float lmax = tr * 0.5f + sqrtf(disc > 0 ? disc : 0);
  Arena a;
  a.valida = valida;
  a.cx = mx; a.cy = my;
  a.dist = hypotf(mx, my);
  a.ang = atan2f(my, mx) * (float)RAD_TO_DEG;
  a.sigma = sqrtf(lmax > 0 ? lmax : 0);
  a.confiavel = valida && a.sigma <= ARENA_SIGMA_OK;
  // amostra das hipóteses, sorteada pelo peso (reamostragem sistemática)
  float passo = sw / ARENA_AMOSTRA, u = aleat() * passo, acum = pw[0];
  int i = 0;
  for (int k = 0; k < ARENA_AMOSTRA; k++) {
    float alvo = u + k * passo;
    while (alvo > acum && i < ARENA_N - 1) acum += pw[++i];
    a.ax[k] = px[i]; a.ay[k] = py[i];
  }
  TRAVA(); saida = a; DESTRAVA();
}

void arena_init() { valida = false; TRAVA(); saida = Arena{}; DESTRAVA(); tAnt = micros(); }

static float escalaInicial() {
  float e = 1.0f + ARENA_RUIDO_MOV * gauss();
  return e < 0.6f ? 0.6f : (e > 1.4f ? 1.4f : e);
}

void arena_inicia(bool opVisto, float opX, float opY) {
  semente ^= micros() | 1u;
  for (int k = 0; k < ARENA_N; k++) {
    if (!sorteiaInicio(px[k], py[k], opVisto, opX, opY)) sorteiaInicio(px[k], py[k], false, 0, 0);
    pw[k] = 1.0f / ARENA_N;
    sv[k] = escalaInicial(); sg[k] = escalaInicial();
  }
  valida = true;
  usPior = 0; tSurpresa = 0; contatoAnt = false;
  accDx = accDth = accDt = 0; accContato = false; accBranco = 0; accPreto = 0xFFFF; ultBranco = 0;
  tAnt = micros(); tPasso = millis();
  calculaSaida();
}

// Predição: o robô andou dx (mm) para a frente e girou dth (rad) em dt segundos; o centro do
// dojô, parado no mundo, anda o contrário no referencial do robô. Em contato o modelo das rodas
// não vale (o robô pode estar travado, sendo empurrado ou sendo girado pelo oponente), e cada
// partícula sorteia um movimento entre ser empurrado para trás a até 250 mm/s e andar o que o
// modelo diz, com giro incerto. A incerteza cresce enquanto o contato dura, como deve.
static void sorteiaContato(int k) {
  ca[k] = -0.5f + 1.6f * aleat();             // de 50% para trás (empurrado) a 110% do avanço pedido
  cg[k] = -0.3f + 1.5f * aleat();             // de girar ao contrário a 120% do giro pedido
  ex[k] = -250.0f + 350.0f * aleat();         // empurrão para trás (até 250 mm/s) ou arrasto para a frente
  ey[k] = (aleat() * 2.0f - 1.0f) * 200.0f;   // empurrão de lado
}

static void predicao(float dx, float dth, bool contato, float dt) {
  if (contato && !contatoAnt)
    for (int k = 0; k < ARENA_N; k++) sorteiaContato(k);
  contatoAnt = contato;
  for (int k = 0; k < ARENA_N; k++) {
    float dxk, dthk, dyk = 0;
    if (contato) {
      dxk  = dx * ca[k] + ex[k] * dt + (fabsf(dx) * 0.1f + 0.5f) * tri();
      dyk  = ey[k] * dt + 0.5f * tri();
      dthk = dth * cg[k] + (fabsf(dth) * 0.1f + 0.01f) * tri();
    } else {
      // ruído de um passo: 5% do movimento mais um piso pequeno, com a distribuição triangular
      // (mais barata que a normal e suficiente para manter as partículas diferentes)
      dxk  = dx * sv[k] + (fabsf(dx) * 0.05f + 0.3f) * tri();
      dthk = dth * sg[k] + (fabsf(dth) * 0.05f + 0.002f) * tri();
    }
    float x = px[k] - dxk, y = py[k] - dyk;
    // Giro de um passo pequeno (menos de 0,5 rad mesmo com o loop atrasado): série de Taylor no
    // lugar de cosf e sinf, com erro abaixo de 1e-4, para caber o dobro de partículas no tempo.
    float t2 = dthk * dthk;
    float c = 1.0f - t2 * (0.5f - t2 * (1.0f / 24.0f));
    float s = dthk * (1.0f - t2 * (1.0f / 6.0f - t2 * (1.0f / 120.0f)));
    px[k] = c * x + s * y;
    py[k] = -s * x + c * y;
  }
}

// Correção com os sensores de borda. Sensor confirmado no branco: a partícula precisa colocar o
// sensor na faixa. Sensor no preto (leitura crua): a partícula não pode colocar o sensor na
// faixa. Sensor com branco ainda sem confirmação não entra (pode ser arranhão). Fora do dojô o
// TCRT5000 também lê escuro, então o preto sozinho não diferencia "dentro" de "fora"; quem
// diferencia é a regra da luta: o robô ainda está no dojô, então as duas rodas estão dentro dele
// e nenhum sensor passa muito da borda.
static void correcao(uint16_t cru, uint16_t confirmado) {
  const float meiaBitola = BITOLA_MM / 2.0f, rRoda2 = R_DOJO * R_DOJO;
  // limites ao quadrado: a distância só sai da raiz quando o sensor viu branco
  const float rFaixaIn2 = (R_FAIXA - 6) * (R_FAIXA - 6), rFaixaOut2 = (R_DOJO + 4) * (R_DOJO + 4);
  const float rAlem2 = (R_DOJO + 20) * (R_DOJO + 20);
  float tot = 0, somaPeso = 0, somaBranco = 0;
  for (int k = 0; k < ARENA_N; k++) {
    // O preto chega a cada 5 ms quase igual, e a raiz suaviza o peso dessas medidas repetidas
    // (elas não são independentes): os fatores do preto abaixo já são as raízes de 0,02, 0,3 e
    // 0,97. O branco é raro e diz muito: entra com o peso inteiro.
    float likPreto = 1.0f, likBranco = 1.0f, likFica = 1.0f;
    float d1 = px[k] * px[k] + (meiaBitola - py[k]) * (meiaBitola - py[k]);
    float d2 = px[k] * px[k] + (meiaBitola + py[k]) * (meiaBitola + py[k]);
    if (d1 > rRoda2 || d2 > rRoda2) likPreto *= 0.1414f;        // roda fora do dojô: o robô já teria caído
    for (uint8_t j = 0; j < N_BORDA; j++) {
      bool branco = (confirmado >> j) & 1, preto = !((cru >> j) & 1);
      if (!branco && !preto) continue;
      float ex = BORDA[j].x - px[k], ey = BORDA[j].y - py[k];
      float r2 = ex * ex + ey * ey;
      bool naFaixa = r2 >= rFaixaIn2 && r2 <= rFaixaOut2;
      bool antes = (ultBranco >> j) & 1;
      // O branco usa uma mistura: um pico estreito (8 mm) onde o sensor deve estar e uma cauda
      // larga (LARGO mm). A cauda faz diferença quando nenhuma hipótese explica o branco: em vez
      // de todas receberem o mesmo peso baixo (e o filtro ficar parado no erro), as mais perto da
      // explicação ganham mais peso e a nuvem anda na direção certa [THRUN; BURGARD; FOX, 2005,
      // cap. 6, modelo de mistura].
      if (branco && !antes) {
        // o sensor acabou de entrar no branco: ele está no começo da faixa, a uns 3 mm dela
        float d = sqrtf(r2) - (R_FAIXA + 3), u = d * (1.0f / 8.0f), w = d * (1.0f / LARGO);
        likBranco *= 0.02f + 0.85f * expf(-0.5f * u * u) + 0.13f * expf(-0.5f * w * w);
      } else if (branco) {
        if (naFaixa) { likBranco *= 0.9f; likFica *= 0.9f; }
        else {
          float r = sqrtf(r2), d = r < R_FAIXA - 6 ? (R_FAIXA - 6) - r : r - (R_DOJO + 4), w = d * (1.0f / LARGO);
          float l = 0.02f + 0.3f * expf(-0.5f * w * w);
          likBranco *= l; likFica *= l;
        }
      } else if (antes) {
        // o sensor acabou de sair do branco para dentro: está logo antes do começo da faixa
        float d = sqrtf(r2) - (R_FAIXA - 3), u = d * (1.0f / 8.0f), w = d * (1.0f / LARGO);
        likBranco *= 0.03f + 0.85f * expf(-0.5f * u * u) + 0.12f * expf(-0.5f * w * w);
      } else {
        likPreto *= (naFaixa || r2 > rAlem2) ? 0.5477f : 0.9849f;  // sensor na faixa ou bem além da borda: pouco provável
      }
    }
    somaPeso += pw[k]; somaBranco += pw[k] * likFica;
    pw[k] *= likPreto * likBranco;
    tot += pw[k];
  }
  // Concordância do branco que continua: peso médio que as hipóteses davam a um sensor que já
  // estava no branco e continua nele. Abaixo de ARENA_SURPRESA com o filtro confiável, a nuvem
  // tirou o sensor da faixa sem o robô sair dela, sinal de que o movimento real fugiu do modelo
  // das rodas (robô preso, patinando ou empurrado de lado sem contato visto pelos ToF). Nesse
  // caso a predição usa o modelo incerto por ARENA_T_SURPRESA_MS, e a nuvem abre para o branco
  // reposicionar as hipóteses (ruído de processo adaptativo, pela inovação da medida).
  if ((confirmado & ultBranco) && saida.sigma <= ARENA_SIGMA_OK && somaPeso > 0 && somaBranco < ARENA_SURPRESA * somaPeso)
    tSurpresa = millis() | 1;
  if (!(tot > 1e-20f)) {                // nada explica a borda: o filtro recomeça largo
    for (int k = 0; k < ARENA_N; k++) {
      float d = aleat() * (R_DOJO - 60), a = (aleat() * 2 - 1) * (float)PI;
      px[k] = d * cosf(a); py[k] = d * sinf(a); pw[k] = 1.0f / ARENA_N;
      sv[k] = escalaInicial(); sg[k] = escalaInicial();
    }
    return;
  }
  float n2 = 0;
  for (int k = 0; k < ARENA_N; k++) { pw[k] /= tot; n2 += pw[k] * pw[k]; }
  // reamostragem sistemática quando o número efetivo de partículas cai abaixo da metade
  if (1.0f / n2 < ARENA_N * 0.5f) {
    float passo = 1.0f / ARENA_N, u = aleat() * passo, acum = pw[0];
    int i = 0;
    for (int k = 0; k < ARENA_N; k++) {
      float alvo = u + k * passo;
      while (alvo > acum && i < ARENA_N - 1) acum += pw[++i];
      qx[k] = px[i] + gauss() * 6.0f;
      qy[k] = py[i] + gauss() * 6.0f;
      qv[k] = sv[i] + gauss() * ARENA_SORTEIO_ESCALA;
      qg[k] = sg[i] + gauss() * ARENA_SORTEIO_ESCALA;
      qca[k] = ca[i] + 0.05f * tri(); qcg[k] = cg[i] + 0.05f * tri();
      qex[k] = ex[i] + 15.0f * tri(); qey[k] = ey[i] + 15.0f * tri();
    }
    // Sem partículas novas sorteadas no dojô inteiro: uma hipótese nova perto do centro explica
    // qualquer preto e, na simulação, roubou a nuvem certa quando o robô ficou preso na borda com
    // o oponente atrás. A recuperação de um erro vem da cauda larga do modelo do branco e do
    // recomeço do filtro quando nada explica a borda.
    for (int k = 0; k < ARENA_N; k++) {
      px[k] = qx[k]; py[k] = qy[k]; pw[k] = 1.0f / ARENA_N;
      ca[k] = qca[k]; cg[k] = qcg[k]; ex[k] = qex[k]; ey[k] = qey[k];
      sv[k] = qv[k] < 0.5f ? 0.5f : (qv[k] > 1.5f ? 1.5f : qv[k]);
      sg[k] = qg[k] < 0.5f ? 0.5f : (qg[k] > 1.5f ? 1.5f : qg[k]);
    }
  }
}

void arena_atualiza(float v_mms, float w_dps, uint16_t bordaCru, uint16_t bordaConf, bool contato) {
  uint32_t agora = micros();
  float dt = (agora - tAnt) * 1e-6f;
  tAnt = agora;
  if (!valida) return;
  if (dt > 0.05f) dt = 0.05f;
  accDx += v_mms * dt;
  accDth += w_dps * (float)DEG_TO_RAD * dt;
  accDt += dt;
  accContato = accContato || contato;
  // A borda é lida em toda volta do loop, mas o filtro anda a cada ARENA_PASSO_MS. Um toque
  // rápido na faixa (a fuga começa na hora) não pode se perder entre dois passos: o branco
  // confirmado em qualquer volta vale, e o preto só vale se ficou preto o tempo todo.
  accBranco |= bordaConf;
  accPreto &= (uint16_t)~bordaCru;
  if (millis() - tPasso < ARENA_PASSO_MS) return;
  tPasso = millis();
  uint32_t t0 = micros();
  bool surpresa = tSurpresa && millis() - tSurpresa < ARENA_T_SURPRESA_MS;
  predicao(accDx, accDth, accContato || surpresa, accDt);
  correcao((uint16_t)~accPreto, accBranco);
  ultBranco = accBranco;
  calculaSaida();
  usPasso = micros() - t0;
  if (usPasso > usPior) usPior = usPasso;
  accDx = accDth = accDt = 0; accContato = false; accBranco = 0; accPreto = 0xFFFF;
}

void arena_le(Arena& a) { TRAVA(); a = saida; DESTRAVA(); }
uint32_t arena_us() { return usPasso; }
uint32_t arena_us_max() { return usPior; }

// Cada hipótese k do centro do dojô (ax, ay) dá a distância t em que o raio que sai do robô na
// direção u deixa o círculo de raio R + margem: t = u.c + raiz((u.c)^2 - |c|^2 + (R + margem)^2).
// O limite é o (ARENA_DISCORDA + 1)-ésimo maior t: além dele, só ARENA_DISCORDA hipóteses ainda
// colocam o ponto dentro.
float arena_limite(const Arena& a, float ux, float uy) {
  if (!a.valida || a.sigma > ARENA_SIGMA_DESCARTE) return 1e9f;
  const float rm = R_DOJO + ARENA_MARGEM_MM, rm2 = rm * rm;
  float maior[ARENA_DISCORDA + 1];
  for (int q = 0; q <= ARENA_DISCORDA; q++) maior[q] = 0;
  for (int k = 0; k < ARENA_AMOSTRA; k++) {
    float uc = ux * a.ax[k] + uy * a.ay[k];
    float disc = uc * uc - (a.ax[k] * a.ax[k] + a.ay[k] * a.ay[k]) + rm2;
    float t = disc > 0 ? uc + sqrtf(disc) : 0;
    for (int q = 0; q <= ARENA_DISCORDA; q++)            // os maiores, em ordem decrescente
      if (t > maior[q]) { float x = maior[q]; maior[q] = t; t = x; }
  }
  return maior[ARENA_DISCORDA];
}

bool arena_fora(const Arena& a, float x, float y) {
  float d = sqrtf(x * x + y * y);
  if (d < 1.0f) return false;
  return d > arena_limite(a, x / d, y / d);
}

float arena_livre(float angGraus) {
  Arena s;
  arena_le(s);
  if (!s.valida) return R_DOJO;
  float a = angGraus * (float)DEG_TO_RAD;
  float ux = cosf(a), uy = sinf(a);
  float uc = ux * s.cx + uy * s.cy;
  float c2 = s.cx * s.cx + s.cy * s.cy;
  float disc = uc * uc - c2 + R_DOJO * R_DOJO;
  return disc > 0 ? uc + sqrtf(disc) : 0;
}
