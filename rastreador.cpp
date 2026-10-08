// rastreador.cpp
// Filtro bayesiano em grade polar para um oponente quadrado de 152 x 152 mm.
// O vetor p[] é o mapa: p[anel * NB + direção] guarda a probabilidade de o centro do
// oponente estar naquela célula, e a soma de todas as células é 1. Tudo o que depende só
// da geometria (o que cada sensor deveria ler para cada célula) é calculado uma vez no
// rastreador_init; durante a luta sobram multiplicações com tabelas, sem trigonometria.
// O filtro custa alguns ms por passo, então ele roda numa tarefa do FreeRTOS no núcleo 0
// (RASTREADOR_NUCLEO), enquanto o loop do robô continua no núcleo 1. O oponente.cpp entrega
// as medidas pelo rastreador_passo e lê o resultado pelo rastreador_estimativa.

#include "rastreador.h"
#include "config.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NB    RAST_NB             // número de direções (3 graus cada)
#define NR    RAST_NR             // número de anéis (20 mm cada), de 150 mm até o alcance
#define NC    (NB * NR)           // número total de células
#define PAD   60                  // folga para a convolução circular (no máximo meia volta)
static const float DB   = 360.0f / NB;
static const float R0   = 150.0f;                    // centro do primeiro anel (mm)
static const float DR   = 20.0f;
static const float MEIO_CONE = 12.5f;                // VL53L0X: cone de 25 graus
// Para um quadrado de lado L, o ToF devolve mais ou menos a face mais próxima dentro do
// cone; na média das orientações isso fica a 0,53 L do centro (80 mm para 152 mm).
static const float RE   = OPONENTE_LADO_MM * 0.526f; // centro até a face "vista"
static const float RP   = OPONENTE_LADO_MM * 0.7071f;// centro até o canto (meia diagonal)
static const float RL   = OPONENTE_LADO_MM * 0.46f;  // folga mínima até o corpo do robô
static const float C0   = 0.02f / TOF_ALCANCE_MM;    // chance de leitura estranha (reflexo ou ruído)
static const float MISTURA = 0.003f;                 // 0,3% espalhado no mapa para reencontrar o oponente se o mapa errar
// Existência, por passo de 20 ms. Na simulação a arena vazia fica com existe perto de 0,03,
// uma leitura falsa isolada chega no máximo a 0,07, o oponente de verdade é confirmado em
// 2 ou 3 medidas e, tirado da frente, some em uns 0,1 s.
static const float P_APARECE = 0.002f;               // chance de alguém colocar o oponente (por passo)
static const float P_SOME    = 0.01f;                // chance de ele sair do alcance sem ser visto (por passo)
static const float GIRO_RUIDO_DPS = 30.0f;           // incerteza mínima do giro quando ele é conhecido
#define Q_INVIS 255                                  // marca de célula que o sensor não enxerga
static const float K_RAD = 0.017453293f, K_GRAU = 57.29578f;   // constantes em float, porque o ESP32 não tem FPU de double

static float*    p     = nullptr;   // mapa (NC células)
static float*    q     = nullptr;   // rascunho do mesmo tamanho
static uint8_t*  eq    = nullptr;   // [sensor][célula] leitura esperada dividida por 4 mm (255 não vê)
static uint16_t* must  = nullptr;   // [célula] bit i ligado quando o centro cai dentro do cone do sensor i
static uint8_t*  livre = nullptr;   // [célula] 1 quando o oponente cabe ali sem invadir o corpo do robô
static float     nLivre = 1;        // quantidade de células livres

static float     Lno[512];                  // tabela sem leitura, indexada por [e/4 + 256*must]
static float     Lz[N_TOF][256];            // tabela com leitura, montada a cada passo
static float     ampK[255], invSgK[255];    // por distância esperada 4k mm: pd/(sg*raiz(2pi)) e 1/sg
static float     grausPorMm[NR];            // quantos graus 1 mm de lado representa no anel r
static float     alcS[N_TOF];               // alcance de cada sensor
static float     wCentro[NB];               // parte de cada direção que cai em ±CENTRO_DEG
static float     covFrac[NB];               // parte dos anéis de cada direção que algum cone vê
static float     swp[NB];                   // rascunho da busca (cobertura acumulada durante o giro)
static float     ext[NB + 2 * PAD];         // anel estendido com as pontas repetidas (convolução circular)
static float     kern[2 * PAD + 2];         // núcleo gaussiano da convolução

static Estimativa est;                      // estimativa atual (só a tarefa escreve)
static float      rExiste = 0;              // probabilidade de o oponente estar no alcance
static uint32_t   passos = 0;               // contador de passos, usado para mandar o mapa a cada 5

// ==================== troca de dados entre o loop (núcleo 1) e a tarefa (núcleo 0) ====================
// A trava (spinlock) protege as variáveis lidas pelos dois núcleos ao mesmo tempo.
#if defined(ARDUINO_ARCH_ESP32) && RASTREADOR_NUCLEO >= 0
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
static portMUX_TYPE trava = portMUX_INITIALIZER_UNLOCKED;
#define TRAVA()    portENTER_CRITICAL(&trava)
#define DESTRAVA() portEXIT_CRITICAL(&trava)
static TaskHandle_t tarefa = nullptr;
#else
#define TRAVA()
#define DESTRAVA()
#endif

// pacote de medidas que o loop deixa para a tarefa processar
struct Entrada {
  int16_t z[N_TOF];
  bool    novo[N_TOF];
  float   angGiro;        // graus girados desde o último passo (quando o giro é conhecido)
  float   dt;
  float   avanco;
  bool    conhecido;
  bool    tem;            // existem medidas esperando
  bool    zera;           // pedido para recomeçar o mapa
};
static Entrada    pend;                         // escrito pelo loop
static Estimativa estPub;                       // publicado pela tarefa
static char       mapaPub[MAPA_NB * MAPA_NR + 4];
static uint16_t   mapaN = 0;
static uint32_t   usUlt = 0, usMax = 0;
static uint16_t   fazMapa(char* buf, uint16_t n);

// Maior leitura do sensor i dentro do círculo de raio TOF_ALCANCE_MM em volta do centro.
// A conta resolve |s + z*u| = alcance, com s a posição do sensor e u a direção dele.
float rastreador_alcance_sensor(uint8_t i) {
  if (i >= N_TOF) return 0;
  float ux = cosf(TOF[i].ang * K_RAD), uy = sinf(TOF[i].ang * K_RAD);
  float su = TOF[i].x * ux + TOF[i].y * uy, ss = (float)TOF[i].x * TOF[i].x + (float)TOF[i].y * TOF[i].y;
  float d = su * su - ss + (float)TOF_ALCANCE_MM * TOF_ALCANCE_MM;
  float z = d > 0 ? -su + sqrtf(d) : 0;
  return z > TOF_MIN_MM ? z : TOF_MIN_MM;
}

// centro em graus da direção b e centro em mm do anel r
static inline float dirCentro(int b) { return -180.0f + DB * 0.5f + DB * b; }
static inline float anelCentro(int r) { return R0 + DR * r; }

// Probabilidade de o ToF enxergar um robô preto a e mm (o preto devolve pouca luz).
static float pd(float e) {
  const float P = ALCANCE_PRETO_MM;
  if (e <= 0.7f * P) return 0.95f;
  float v = 0.95f - 0.85f * (e - 0.7f * P) / (0.45f * P);
  return v < 0.05f ? 0.05f : (v > 0.95f ? 0.95f : v);
}

// Leitura esperada do sensor i com o centro do oponente em (cx, cy); o retorno -1 indica
// que o sensor não enxerga essa posição. O ponteiro dentro informa se o centro está dentro
// do cone (nesse caso o sensor teria que ver). Rv é até onde, além da borda do cone, o
// oponente ainda aparece (RE para o mapa e RP para conferir a máscara).
static float esperado(uint8_t i, float cx, float cy, float Rv, bool* dentro) {
  float dx = cx - TOF[i].x, dy = cy - TOF[i].y;
  float dd = sqrtf(dx * dx + dy * dy);
  float beta = atan2f(dy, dx) * K_GRAU - TOF[i].ang;
  // os dois laços deixam o ângulo entre -180 e 180
  while (beta > 180.0f) beta -= 360.0f;
  while (beta < -180.0f) beta += 360.0f;
  float ab = fabsf(beta);
  bool in = ab <= MEIO_CONE;
  if (dentro) *dentro = in;
  if (dd >= alcS[i] + RP) return -1;
  float e;
  if (in) e = dd - RE;                                  // de frente: o sensor vê a face mais próxima
  else {                                                // fora do cone: só a lateral do oponente entra
    float delta = (ab - MEIO_CONE) * K_RAD;
    if (delta >= (float)M_PI / 2) return -1;            // posição atrás do sensor
    float pp = dd * sinf(delta);                        // distância do centro até a borda do cone
    if (pp >= Rv) return -1;
    float h = RE * RE - pp * pp;
    e = dd * cosf(delta) - (h > 0 ? sqrtf(h) : 0.0f);
  }
  return e < 5.0f ? 5.0f : e;
}

// liberação da memória quando algum malloc falha
static void libera() {
  free(p); free(q); free(eq); free(must); free(livre);
  p = q = nullptr; eq = nullptr; must = nullptr; livre = nullptr;
}

// mapa uniforme nas células livres e existência no valor de equilíbrio
static void zeraMapa() {
  for (int c = 0; c < NC; c++) p[c] = livre[c] / nLivre;
  rExiste = P_APARECE / (P_APARECE + P_SOME);
  memset(&est, 0, sizeof(est));
  est.distSensor = -1;
  est.existe = rExiste;
}

// Alocação e pré-cálculo de todas as tabelas; chamada uma vez pelo tof_init (oponente.cpp).
bool rastreador_init() {
  if (p) return true;
  p     = (float*)malloc(NC * sizeof(float));
  q     = (float*)malloc(NC * sizeof(float));
  eq    = (uint8_t*)malloc((size_t)N_TOF * NC);
  must  = (uint16_t*)malloc(NC * sizeof(uint16_t));
  livre = (uint8_t*)malloc(NC);
  if (!p || !q || !eq || !must || !livre) { libera(); return false; }

  for (uint8_t i = 0; i < N_TOF; i++) alcS[i] = rastreador_alcance_sensor(i);
  nLivre = 0;
  // laço duplo pela grade (anel e direção): para cada célula o código calcula se ela está
  // livre e o que cada sensor leria com o oponente centrado ali
  for (int r = 0; r < NR; r++) {
    float rho = anelCentro(r);
    for (int b = 0; b < NB; b++) {
      int c = r * NB + b;
      float th = dirCentro(b) * K_RAD;
      float cx = rho * cosf(th), cy = rho * sinf(th);
      // o oponente não pode estar dentro do corpo do robô: ponto do retângulo mais perto da célula
      float qx = cx > CORPO_FRENTE_MM ? CORPO_FRENTE_MM : (cx < -CORPO_TRAS_MM ? -CORPO_TRAS_MM : cx);
      float qy = cy > CORPO_ESQ_MM ? CORPO_ESQ_MM : (cy < -CORPO_DIR_MM ? -CORPO_DIR_MM : cy);
      livre[c] = hypotf(cx - qx, cy - qy) >= RL ? 1 : 0;
      nLivre += livre[c];
      uint16_t m = 0;
      for (uint8_t i = 0; i < N_TOF; i++) {
        bool dentro;
        float e = esperado(i, cx, cy, RE, &dentro);
        uint8_t v = Q_INVIS;
        if (e >= 0) { int k = (int)lroundf(e / 4.0f); v = k > 254 ? 254 : k; }   // valor guardado em passos de 4 mm
        eq[(size_t)i * NC + c] = v;
        if (v != Q_INVIS && dentro && e < alcS[i]) m |= (1u << i);
      }
      must[c] = m;
    }
  }
  if (nLivre < 1) nLivre = 1;

  // Tabela sem leitura: no meio do cone (must) o oponente quase certamente seria visto; na
  // borda do cone a chance é menor; longe e preto ele pode passar despercebido (pd baixo).
  for (int k = 0; k < 256; k++) {
    float d = pd(4.0f * k);
    Lno[k]       = k == Q_INVIS ? 1.0f : (1.0f - 0.4f * d) * 0.98f + 0.002f;
    Lno[256 + k] = k == Q_INVIS ? 1.0f : (1.0f - d) * 0.98f + 0.002f;
  }
  // Tabela com leitura: o ruído do ToF cresce com a distância (sg = 12 mm + 3%)
  for (int k = 0; k < 255; k++) {
    float e = 4.0f * k, sg = 12.0f + 0.03f * e;
    invSgK[k] = 1.0f / sg;
    ampK[k] = pd(e) / (sg * 2.5066283f);
  }
  for (int r = 0; r < NR; r++) grausPorMm[r] = K_GRAU / anelCentro(r);
  for (int b = 0; b < NB; b++) {                    // parte de cada direção que cai em ±CENTRO_DEG
    float a0 = dirCentro(b) - DB / 2, a1 = a0 + DB;
    float lo = a0 > -CENTRO_DEG ? a0 : -CENTRO_DEG, hi = a1 < CENTRO_DEG ? a1 : CENTRO_DEG;
    wCentro[b] = hi > lo ? (hi - lo) / DB : 0.0f;
  }
  // cobertura dos cones por direção, usada na busca: 1 quando todos os anéis livres estão em algum cone
  for (int b = 0; b < NB; b++) {
    int n = 0, v = 0;
    for (int r = 0; r < NR; r++) { int c = r * NB + b; if (!livre[c]) continue; n++; if (must[c]) v++; }
    covFrac[b] = n ? (float)v / n : 0.0f;
  }
  zeraMapa();
  memset(&pend, 0, sizeof(pend));
  estPub = est;
#if defined(ARDUINO_ARCH_ESP32) && RASTREADOR_NUCLEO >= 0
  // tarefa de prioridade baixa no outro núcleo, que fica livre porque o firmware não usa o rádio
  extern void rastreadorTarefa(void*);
  if (xTaskCreatePinnedToCore(rastreadorTarefa, "rastreador", 4096, nullptr, 1, &tarefa, RASTREADOR_NUCLEO) != pdPASS)
    tarefa = nullptr;                             // sem tarefa o filtro roda no próprio loop
#endif
  return true;
}

// ==================== etapa 1: predição ====================
// índice de anel refletido nas pontas, para a probabilidade não escapar da grade
static inline int reflete(int r) { return r < 0 ? -r - 1 : (r >= NR ? 2 * NR - r - 1 : r); }

static void predicao(float giro, bool conhecido, float avanco, float dt) {
  // deslocamento máximo do oponente em relação ao robô neste passo
  float sp = (OP_VEL_MAX_MMS + fabsf(avanco)) * dt;

  // parte (a), distância: o mapa borra entre anéis com um núcleo gaussiano
  float sr = sp / DR;
  int kr = (int)ceilf(2.5f * sr);
  if (kr > 12) kr = 12;
  if (kr < 1) kr = 1;
  float soma = 0;
  const float isr = 1.0f / sr;
  for (int j = -kr; j <= kr; j++) { float u = j * isr; kern[j + kr] = expf(-0.5f * u * u); soma += kern[j + kr]; }
  soma = 1.0f / soma;
  for (int j = 0; j <= 2 * kr; j++) kern[j] *= soma;   // normalização do núcleo para a soma dar 1
  // para cada anel, soma os anéis vizinhos com o peso do núcleo (resultado em q)
  for (int r = 0; r < NR; r++) {
    float* dst = q + r * NB;
    memset(dst, 0, NB * sizeof(float));
    for (int j = -kr; j <= kr; j++) {
      const float w = kern[j + kr];
      const float* src = p + reflete(r - j) * NB;
      for (int b = 0; b < NB; b++) dst[b] += w * src[b];
    }
  }

  // Parte (b), direção, anel por anel: o oponente andando de lado (perto vale mais graus)
  // mais o giro do robô. Com giro conhecido (motores) o mapa gira ao contrário; com giro
  // desconhecido (mão) o mapa só borra.
  float desloc = conhecido ? -giro * dt * (1.0f / DB) : 0.0f;       // deslocamento em direções
  float srot = (conhecido ? 0.3f * fabsf(giro) + GIRO_RUIDO_DPS : (float)GIRO_DESCONHECIDO_DPS) * dt;
  for (int r = 0; r < NR; r++) {
    float sb = (sp * grausPorMm[r] + srot) * (1.0f / DB);
    if (sb < 0.15f) sb = 0.15f;
    const float isb = 1.0f / sb;
    int jlo = (int)floorf(desloc - 2.5f * sb), jhi = (int)ceilf(desloc + 2.5f * sb);
    if (jlo < -PAD + 1) jlo = -PAD + 1;
    if (jhi > PAD - 1) jhi = PAD - 1;
    soma = 0;
    for (int j = jlo; j <= jhi; j++) { float u = (j - desloc) * isb; kern[j - jlo] = expf(-0.5f * u * u); soma += kern[j - jlo]; }
    soma = 1.0f / soma;
    for (int j = 0; j <= jhi - jlo; j++) kern[j] *= soma;
    // o anel vira um vetor estendido com as pontas repetidas, assim a volta de 360 graus não precisa de módulo
    const float* src = q + r * NB;
    memcpy(ext + PAD, src, NB * sizeof(float));                     // ext[PAD + b] = src[b]
    memcpy(ext, src + NB - PAD, PAD * sizeof(float));
    memcpy(ext + PAD + NB, src, PAD * sizeof(float));
    float* dst = p + r * NB;
    memset(dst, 0, NB * sizeof(float));
    for (int j = jlo; j <= jhi; j++) {                               // dst[b] += w_j * src[b - j]
      const float w = kern[j - jlo];
      const float* e = ext + PAD - j;
      for (int b = 0; b < NB; b++) dst[b] += w * e[b];
    }
  }

  // parte (c), regra: célula dentro do corpo do robô vale zero; o total serve para normalizar
  float tot = 0;
  for (int c = 0; c < NC; c++) { if (!livre[c]) p[c] = 0; tot += p[c]; }
  if (!(tot > 1e-30f)) { zeraMapa(); return; }
  // Parte (d), existência: o oponente pode sumir (sair do alcance ou ser tirado) ou
  // aparecer em qualquer lugar livre. O mapa p[] continua sendo "onde está, se estiver".
  const float fica = rExiste * (1.0f - P_SOME), nasce = (1.0f - rExiste) * P_APARECE, rn = fica + nasce;
  const float a = (1.0f - MISTURA) * fica / rn / tot, u = (MISTURA * fica + nasce) / rn / nLivre;
  for (int c = 0; c < NC; c++) if (livre[c]) p[c] = a * p[c] + u;
  rExiste = rn;
}

// ==================== etapa 2: correção ====================
// Cada sensor com medida nova confirma ou desmente cada célula do mapa.
static void correcao(const int16_t* z, const bool* novo) {
  const float* tab[N_TOF];
  const uint8_t* lin[N_TOF];
  uint8_t sh[N_TOF];
  uint8_t nu = 0;
  float La = 1.0f;                               // chance destas medidas sem oponente nenhum (só reflexo ou ruído)
  // primeiro laço: escolhe a tabela de cada sensor (com leitura ou sem leitura)
  for (uint8_t i = 0; i < N_TOF; i++) {
    if (!novo[i]) continue;
    if (z[i] > 0) {
      // com leitura z: chance de ler z com o oponente na posição da célula (e é a leitura esperada)
      const float zz = z[i];
      for (int k = 0; k < 255; k++) {
        float u = (zz - 4.0f * k) * invSgK[k];
        Lz[i][k] = (u > 6.0f || u < -6.0f) ? C0 : ampK[k] * expf(-0.5f * u * u) + C0;
      }
      Lz[i][Q_INVIS] = C0;                       // célula que o sensor não enxerga: a leitura é de outra coisa
      tab[nu] = Lz[i]; sh[nu] = 16;              // com leitura o must não importa (deslocar 16 bits zera)
      La *= C0;
    } else {
      tab[nu] = Lno; sh[nu] = i;
    }
    lin[nu] = eq + (size_t)i * NC;
    nu++;
  }
  if (!nu) return;
  // segundo laço: multiplica cada célula pelas chances de todos os sensores (regra de Bayes)
  float tot = 0;
  for (int c = 0; c < NC; c++) {
    float v = p[c];
    if (v == 0.0f) continue;
    const uint32_t m = must[c];
    for (uint8_t k = 0; k < nu; k++) v *= tab[k][lin[k][c] | (((m >> sh[k]) & 1u) << 8)];
    p[c] = v; tot += v;
  }
  // existência: compara "tem oponente em algum lugar do mapa" com "não tem, foi reflexo ou ruído"
  const float com = rExiste * tot, sem = (1.0f - rExiste) * La;
  if (com + sem > 0) rExiste = com / (com + sem);
  if (rExiste > 0.9999f) rExiste = 0.9999f;
  if (rExiste < 1e-4f) rExiste = 1e-4f;
  if (!(tot > 1e-30f)) {                         // nenhuma célula explica as medidas: o mapa recomeça
    for (int c = 0; c < NC; c++) p[c] = livre[c] / nLivre;
    return;
  }
  const float a = 1.0f / tot;
  for (int c = 0; c < NC; c++) p[c] *= a;        // normalização para a soma voltar a 1
}

// ==================== etapa 3: estimativa ====================
// Pico do mapa e média numa janela em volta dele (±18 graus e ±100 mm).
static void estima(const int16_t* z) {
  int cMax = 0;
  float vMax = -1;
  for (int c = 0; c < NC; c++) if (p[c] > vMax) { vMax = p[c]; cMax = c; }   // busca do pico
  const int ib = cMax % NB, ir = cMax / NB, WB = 6, WR = 5;
  float m = 0, sb = 0, sbb = 0, sr = 0;
  // laço duplo na janela: soma a massa, os momentos do ângulo e a distância média
  for (int dr = -WR; dr <= WR; dr++) {
    int r = ir + dr;
    if (r < 0 || r >= NR) continue;
    for (int db = -WB; db <= WB; db++) {
      int b = (ib + db + NB) % NB;
      float v = p[r * NB + b];
      m += v; sb += v * db; sbb += v * db * db; sr += v * anelCentro(r);
    }
  }
  if (m <= 0) m = 1e-12f;
  float mb = sb / m;
  float ang = dirCentro(ib) + mb * DB;
  if (ang > 180) ang -= 360;
  if (ang < -180) ang += 360;
  float var = sbb / m - mb * mb;
  est.angulo = ang;
  est.rho    = sr / m;
  est.x      = est.rho * cosf(ang * K_RAD);
  est.y      = est.rho * sinf(ang * K_RAD);
  est.sigma  = sqrtf(var > 0 ? var : 0) * DB;
  est.massa  = m;
  est.existe = rExiste;

  // probabilidade de o centro estar na faixa de ataque (±CENTRO_DEG)
  float pc = 0;
  for (int b = 0; b < NB; b++) {
    if (wCentro[b] <= 0) continue;
    float col = 0;
    for (int r = 0; r < NR; r++) col += p[r * NB + b];
    pc += wCentro[b] * col;
  }
  est.pCentro = pc;

  // máscara: sensores cuja leitura combina com a leitura esperada na posição estimada
  est.mascara = 0;
  est.distSensor = -1;
  for (uint8_t i = 0; i < N_TOF; i++) {
    if (z[i] <= 0) continue;
    float e = esperado(i, est.x, est.y, RP, nullptr);
    if (e < 0) continue;
    float tol = 3.0f * (12.0f + 0.03f * e) + 25.0f;
    if (fabsf(z[i] - e) <= tol) {
      est.mascara |= (1u << i);
      if (est.distSensor < 0 || z[i] < est.distSensor) est.distSensor = z[i];
    }
  }
}

// ==================== etapa 4: busca (sem ver o oponente) ====================
// Para cada giro possível (de 6 em 6 graus, até meia volta para cada lado) a função soma
// quanto do mapa os cones varrem durante o giro, e não só onde eles param. O giro escolhido
// é o que cobre mais probabilidade por segundo: o robô olha primeiro os pontos cegos onde
// o oponente provavelmente está e não perde tempo onde os sensores acabaram de ver.
// O resultado vira P.op.busca_ang, usado no rumo (percepcao.cpp) e no estado BUSCA.
static void busca() {
  float Pb[NB];
  for (int b = 0; b < NB; b++) { float s = 0; for (int r = 0; r < NR; r++) s += p[r * NB + b]; Pb[b] = s; }   // probabilidade por direção
  float agora = 0;                                       // o que os cones já cobrem com o robô parado
  for (int b = 0; b < NB; b++) agora += Pb[b] * covFrac[b];
  const float wBusca = GIRO_MAX_DPS * (float)VEL_BUSCA / 255.0f;   // velocidade de giro na busca (graus/s)
  float melhor = -1, melhorAng = 0, melhorGanho = 0;
  // laço externo: os dois sentidos de giro; laço do meio: o tamanho do giro; laço interno: as direções
  for (int sentido = -1; sentido <= 1; sentido += 2) {
    for (int b = 0; b < NB; b++) swp[b] = covFrac[b];
    for (int k = 1; k <= NB / 2; k++) {
      float cob = 0;
      for (int b = 0; b < NB; b++) {
        int j = b - sentido * k; j %= NB; if (j < 0) j += NB;     // depois do giro, o que estava em b passa a b - k
        if (covFrac[j] > swp[b]) swp[b] = covFrac[j];
        cob += Pb[b] * swp[b];
      }
      if (k % 2) continue;                                 // a avaliação só acontece de 6 em 6 graus
      float ang = sentido * k * DB;
      float t = 0.15f + fabsf(ang) / wBusca;              // tempo para girar até lá, mais o tempo de reação
      float score = (cob - agora) / t;
      if (score > melhor) { melhor = score; melhorAng = ang; melhorGanho = cob; }
    }
  }
  est.buscaAng = melhorAng;
  est.buscaGanho = melhorGanho;
}

// Um passo completo do filtro. Ele roda na tarefa do núcleo 0 ou no loop, quando não existe tarefa.
static void processa(const Entrada& in) {
  if (in.zera) zeraMapa();
  if (in.tem) {
    uint32_t t0 = micros();
    float dt = in.dt < 0.005f ? 0.005f : (in.dt > 0.1f ? 0.1f : in.dt);
    float giro = in.dt > 0 ? in.angGiro / in.dt : 0;
    predicao(giro, in.conhecido, in.avanco, dt);
    correcao(in.z, in.novo);
    estima(in.z);
    busca();
    uint32_t us = micros() - t0;
    static char m[MAPA_NB * MAPA_NR + 4];
    uint16_t n = (passos++ % 5 == 0) ? fazMapa(m, sizeof(m)) : 0;      // mapa do painel a cada 5 passos
    // publicação do resultado para o loop dentro da trava
    TRAVA();
    estPub = est;
    usUlt = us;
    if (us > usMax) usMax = us;
    if (n) { memcpy(mapaPub, m, n + 1); mapaN = n; }
    DESTRAVA();
  } else {
    TRAVA(); estPub = est; DESTRAVA();
  }
}

// cópia do pacote pendente e limpeza do original, tudo dentro da trava
static Entrada pegaPendente() {
  TRAVA();
  Entrada in = pend;
  pend.tem = pend.zera = false;
  pend.dt = pend.angGiro = pend.avanco = 0;
  memset(pend.novo, 0, sizeof(pend.novo));
  DESTRAVA();
  return in;
}

#if defined(ARDUINO_ARCH_ESP32) && RASTREADOR_NUCLEO >= 0
// laço infinito da tarefa: ela dorme até o loop avisar que chegaram medidas
void rastreadorTarefa(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);       // a tarefa dorme até o aviso do rastreador_passo
    Entrada in = pegaPendente();
    if (in.tem || in.zera) processa(in);
  }
}
static inline bool temTarefa() { return tarefa != nullptr; }
static inline void acorda() { xTaskNotifyGive(tarefa); }
#else
static inline bool temTarefa() { return false; }
static inline void acorda() {}
#endif

// O oponente.cpp chama esta função a cada 20 ms: as medidas vão para o pacote e a tarefa acorda.
void rastreador_passo(const int16_t* z, const bool* novo, float giro_dps, bool giroConhecido,
                      float avanco_mms, float dt) {
  if (!p) return;
  TRAVA();
  for (uint8_t i = 0; i < N_TOF; i++) {
    if (novo[i]) { pend.novo[i] = true; pend.z[i] = z[i]; }     // medida nova
    else if (!pend.novo[i]) pend.z[i] = z[i];                   // sem medida nova fica a última leitura, só para conferir a máscara
  }
  // se a tarefa atrasar, os giros e os tempos vão somando até o próximo passo
  pend.angGiro += giro_dps * dt;
  pend.dt += dt;
  pend.conhecido = giroConhecido;
  if (fabsf(avanco_mms) > fabsf(pend.avanco)) pend.avanco = avanco_mms;
  pend.tem = true;
  DESTRAVA();
  if (temTarefa()) acorda();
  else processa(pegaPendente());
}

// pedido para recomeçar o mapa (usado quando os sensores são iniciados de novo)
void rastreador_zera() {
  if (!p) return;
  TRAVA(); pend.zera = true; DESTRAVA();
  if (temTarefa()) acorda();
  else processa(pegaPendente());
}

// leituras protegidas pela trava, chamadas pelo oponente.cpp e pelo debug.cpp
Estimativa rastreador_estimativa() { TRAVA(); Estimativa e = estPub; DESTRAVA(); return e; }
uint32_t rastreador_us()      { TRAVA(); uint32_t v = usUlt; DESTRAVA(); return v; }
uint32_t rastreador_us_max()  { TRAVA(); uint32_t v = usMax; DESTRAVA(); return v; }
void     rastreador_us_zera() { TRAVA(); usMax = 0; DESTRAVA(); }
bool     rastreador_paralelo(){ return temTarefa(); }

// ==================== mapa para o painel (blocos de 3 direções por 2 anéis) ====================
// cópia do último mapa comprimido pronto (usada pelo debug.cpp no stream)
uint16_t rastreador_mapa(char* buf, uint16_t n) {
  if (!p || n < 2) return 0;
  TRAVA();
  uint16_t k = mapaN < n ? mapaN : 0;
  if (k) memcpy(buf, mapaPub, k + 1);
  DESTRAVA();
  return k;
}

// montagem do mapa comprimido: soma dos blocos, conversão em nível de 0 a 63 e junção dos zeros seguidos
static uint16_t fazMapa(char* buf, uint16_t n) {
  if (n < 2) return 0;
  static float s[MAPA_NB * MAPA_NR];
  float smax = 1e-12f;
  for (int R = 0; R < MAPA_NR; R++)
    for (int M = 0; M < MAPA_NB; M++) {
      float v = 0;
      for (int r = 2 * R; r < 2 * R + 2 && r < NR; r++)
        for (int b = 3 * M; b < 3 * M + 3; b++) v += p[r * NB + b];
      s[R * MAPA_NB + M] = v;
      if (v > smax) smax = v;
    }
  const float inv = 1.0f / smax;
  uint16_t k = 0;
  int zeros = 0;
  // o laço vai uma posição além do fim para descarregar a última sequência de zeros
  for (int c = 0; c <= MAPA_NB * MAPA_NR; c++) {
    int nivel = 0;
    if (c < MAPA_NB * MAPA_NR) nivel = (int)lroundf(63.0f * sqrtf(s[c] * inv));
    if (nivel == 0 && c < MAPA_NB * MAPA_NR && zeros < 78) { zeros++; continue; }
    // descarga da sequência de zeros: um zero sozinho vira '0', vários viram "!" e a contagem
    if (zeros == 1) { if (k + 1 >= n) break; buf[k++] = '0'; }
    else if (zeros > 1) { if (k + 2 >= n) break; buf[k++] = '!'; buf[k++] = (char)('0' + zeros); }
    zeros = 0;
    if (c == MAPA_NB * MAPA_NR) break;
    if (nivel == 0) { zeros = 1; continue; }        // caso que só acontece quando a sequência chegou a 78
    if (k + 1 >= n) break;
    buf[k++] = (char)('0' + nivel);
  }
  buf[k] = 0;
  return k;
}
