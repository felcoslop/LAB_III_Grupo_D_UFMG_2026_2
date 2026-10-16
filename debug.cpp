// debug.cpp
// Comandos de teste, calibração e o stream de dados para o painel. Os comandos chegam pela
// entrada_le() do telemetria.cpp (Monitor Serial ou painel, os dois pela USB) e o comando help
// mostra a lista completa.
// Este é o único módulo que conversa com todos os outros, porque precisa mostrar tudo.
#include "debug.h"
#include "config.h"
#include "telemetria.h"
#include "percepcao.h"
#include "oponente.h"
#include "borda.h"
#include "motores.h"
#include "estados.h"
#include "rastreador.h"
#include <Wire.h>

static char     linha[48];          // comando que está sendo digitado
static uint8_t  nLinha = 0;
// modos contínuos ligados e desligados pelos comandos, cada um com o horário da última impressão
static bool     modoVer = false, modoId = false, modoBorda = false, modoStream = false;
static uint32_t tVer = 0, tId = 0, tBorda = 0, tStream = 0, tMapa = 0;
static uint16_t cobertoAnt = 0;     // sensores cobertos na leitura anterior do modo id

static void streamDados();
static void streamMapa();

// Lista de comandos (help).
static void ajuda() {
  Log.println(F("# ================= COMANDOS ================="));
  Log.println(F("# lista              sensores configurados e status"));
  Log.println(F("# id                 liga/desliga: cubra um sensor e veja o NOME dele"));
  Log.println(F("# ver                liga/desliga leitura continua (sensores + estado)"));
  Log.println(F("# scan               enderecos I2C presentes no barramento"));
  Log.println(F("# cal NOME [P1 P2]   calibra 2 pontos (padrao 50 e 300 mm), grava no ESP32"));
  Log.println(F("# calver             mostra ganho/offset de cada sensor"));
  Log.println(F("# calzera NOME|todos apaga a calibracao gravada"));
  Log.println(F("# medir NOME MM      confere: media, ruido e erro com alvo a MM"));
  Log.println(F("# borda              liga/desliga leitura dos sensores de borda (mV, limiar)"));
  Log.println(F("# calborda           diagnostico + calibracao da borda: ar, preto e branco"));
  Log.println(F("# bordazera          apaga a calibracao da borda"));
  Log.println(F("# init               tenta religar sensores que falharam (robo parado)"));
  Log.println(F("# tempo              duracao do ciclo (atual e pior caso)"));
  Log.println(F("# go | stop          inicia (5 s) / para a maquina de estados"));
  Log.println(F("# s1 | s0            liga/desliga stream para o painel (HTML)"));
  Log.println(F("# ============================================"));
}

// Comando lista: tabela de sensores do config.h junto com a calibração e o status de cada um.
static void lista() {
  Log.printf("# geometria: %s\n", GEOMETRIA_NOME);
  Log.println(F("# nome  XSHUT   end.   x    y   ang   ganho  offset  cal     status"));
  for (uint8_t i = 0; i < N_TOF; i++)
    Log.printf("# %-4s  GPIO%-2d  0x%02X %4d %4d %4d  %6.3f %+6.1f  %-6s  %s\n", TOF[i].nome, TOF[i].xshut,
                  TOF[i].endereco, TOF[i].x, TOF[i].y, TOF[i].ang, tof_cal_ganho(i), tof_cal_offset(i),
                  tof_cal_gravada(i) ? "ESP32" : "config", tof_ok(i) ? "OK" : "FALHOU");
  Log.printf("# sensor central (decide ataque): %s\n", tof_centro() >= 0 ? TOF[tof_centro()].nome : "-");
  Log.printf("# %u de %u OK -> %s\n", P.saude.tof_ok, P.saude.tof_total,
                P.saude.pronto ? "PRONTO para lutar" : "NAO inicia (minimo TOF_MINIMO_PARA_LUTAR)");
#if USAR_BORDA
  Log.println(F("# borda  GPIO   x    y   limiar  preto  branco  cal"));
  for (uint8_t i = 0; i < N_BORDA; i++)
    Log.printf("# %-4s   %2d  %4d %4d  %5u  %5u  %5u   %s\n", BORDA[i].nome, BORDA[i].pino, BORDA[i].x, BORDA[i].y,
               borda_limiar(i), borda_cal_preto(i), borda_cal_branco(i), borda_cal_gravada(i) ? "ESP32" : "padrao");
  for (uint8_t i = 0; i < N_BORDA; i++)
    if (!borda_cal_gravada(i)) { Log.printf("! borda SEM calibracao: rode calborda (sem ela vale o limiar fixo de %d mV)\n", BORDA_LIMIAR_MV); break; }
#else
  Log.println(F("# borda: desativada (USAR_BORDA 0)"));
#endif
}

// Comando scan: busca de dispositivos no barramento I2C. O laço testa todos os endereços e, quando algum
// responde, tenta achar o nome dele na tabela TOF.
static void scan() {
  Log.println(F("# varrendo I2C..."));
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      const char* nome = "?";
      for (uint8_t i = 0; i < N_TOF; i++) if (TOF[i].endereco == a) nome = TOF[i].nome;
      if (a == 0x29) nome = "0x29 = sensor que NAO recebeu endereco (XSHUT solto?)";
      Log.printf("#   0x%02X  %s\n", a, nome);
      n++;
    }
  }
  Log.printf("# %u dispositivo(s)\n", n);
}

// Esta função mantém os sensores lidos e o painel atualizado enquanto alguma rotina espera o usuário
// (calibração), já que nesse tempo o loop() principal fica parado.
static void servico() {
  percepcao_atualiza();
  uint32_t t = millis();
  if (modoStream && t - tStream >= STREAM_MS) { tStream = t; streamDados(); }
  if (modoStream && oponente_bayes() && t - tMapa >= MAPA_MS) { tMapa = t; streamMapa(); }
}

// A função espera um ENTER (retorno true) ou "x" seguido de ENTER (retorno false), com limite de 120 s.
// Os laços while (entrada_le() >= 0) {} servem para jogar fora o que sobrou na entrada.
static bool esperaEnter() {
  delay(20);
  while (entrada_le() >= 0) {}
  char primeiro = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 120000) {
    servico();
    int ch;
    while ((ch = entrada_le()) >= 0) {
      if (ch == '\r' || ch == '\n') {
        delay(10);
        while (entrada_le() >= 0) {}
        return primeiro != 'x' && primeiro != 'X';
      }
      if (!primeiro && ch != ' ') primeiro = (char)ch;
    }
  }
  Log.println(F("! tempo esgotado"));
  return false;
}

// Média e desvio padrão de n leituras brutas novas do sensor i. O tof_seq() muda a cada medida
// nova, então a mesma leitura não entra duas vezes na conta.
static bool mediaBruta(uint8_t i, uint8_t n, float& media, float& dp) {
  uint16_t ult = tof_seq(i);
  uint8_t k = 0, ruins = 0;
  float s = 0, s2 = 0;
  uint32_t t0 = millis();
  while (k < n && millis() - t0 < 6000) {
    servico();
    if (tof_seq(i) != ult) {
      ult = tof_seq(i);
      int16_t r = tof_bruto(i);
      if (r > 0) { s += r; s2 += (float)r * r; k++; }
      else if (++ruins > n) return false;
    }
  }
  if (k < n) return false;
  media = s / k;
  dp = sqrtf(max(0.0f, s2 / k - media * media));
  return true;
}

// Conversão do nome digitado no índice do sensor, conferindo se ele pode ser usado agora.
static int8_t sensorDoArg(const char* nome) {
  int8_t i = tof_indice(nome);
  if (i < 0) { Log.printf("! sensor '%s' nao existe na tabela TOF\n", nome); return -1; }
  if (!tof_ok(i)) { Log.printf("! %s nao esta respondendo (lista)\n", TOF[i].nome); return -1; }
  if (estado_atual() != ESPERA) { Log.println(F("! pare o robo antes (stop)")); return -1; }
  return i;
}

// Calibração de 2 pontos (comando cal).
// A conta é corrigido = ganho * bruto + offset. Com isso o erro fixo (offset de fábrica, comum
// nos clones, de +10 a +30 mm) e o erro de escala são corrigidos. O resultado vai para o
// oponente.cpp pela tof_cal_define(), que grava na memória do ESP32.
static void calibra(const char* args) {
  char nome[8] = {0};
  int p1 = CAL_P1_MM, p2 = CAL_P2_MM;
  if (sscanf(args, "%7s %d %d", nome, &p1, &p2) < 1) { Log.println(F("! uso: cal FC   ou   cal FC 50 300")); return; }
  if (p1 < 20 || p2 <= p1 + 100 || p2 > 1000) { Log.println(F("! use P1 >= 20 e P2 pelo menos 100 mm maior")); return; }
  int8_t i = sensorDoArg(nome);
  if (i < 0) return;

  Log.printf("# ===== CALIBRANDO %s (2 pontos: %d e %d mm) =====\n", TOF[i].nome, p1, p2);
  Log.println(F("# Alvo PLANO e FOSCO (papelao, ou o material preto do oponente), maior que 5x5 cm,"));
  Log.println(F("# bem DE FRENTE para o sensor. Distancia medida da FACE DO CHIP preto ate o alvo."));
  const int ref[2] = {p1, p2};
  float m[2];
  // o laço externo passa pelos dois pontos e o while repete a medida até ela ficar estável
  for (uint8_t k = 0; k < 2; k++) {
    while (true) {
      Log.printf("> Alvo a %d mm do %s. ENTER mede (x + ENTER cancela).\n", ref[k], TOF[i].nome);
      if (!esperaEnter()) { Log.println(F("! calibracao cancelada, nada foi gravado")); return; }
      float med, dp;
      if (!mediaBruta(i, 50, med, dp)) { Log.println(F("! sem leituras validas: alvo no cone? repita")); continue; }
      Log.printf("# bruto: media %.1f mm, ruido +-%.1f mm\n", med, dp);
      if (dp > (k == 0 ? 6.0f : 12.0f)) { Log.println(F("! instavel: firme o alvo e o sensor, repita")); continue; }
      m[k] = med;
      break;
    }
  }
  if (m[1] - m[0] < 50) { Log.println(F("! as duas medidas ficaram quase iguais: distancias certas? nada gravado")); return; }
  // reta que passa pelos dois pontos medidos
  float g = (float)(p2 - p1) / (m[1] - m[0]);
  float o = p1 - g * m[0];
  if (g < 0.8f || g > 1.25f) {
    Log.printf("! ganho %.3f fora do normal (0,8 a 1,25). Confira as distancias. Nada gravado.\n", g);
    return;
  }
  tof_cal_define(i, g, o);
  Log.printf("# %s CALIBRADO: ganho %.3f, offset %+.1f mm (gravado no ESP32)\n", TOF[i].nome, g, o);
  Log.printf("# confira: \"medir %s 100\" com o alvo a 100 mm\n", TOF[i].nome);
}

// Comando calver: ganho e offset em uso em cada sensor.
static void calver() {
  Log.println(F("# nome   ganho   offset   origem"));
  for (uint8_t i = 0; i < N_TOF; i++)
    Log.printf("# %-4s  %6.3f  %+6.1f   %s\n", TOF[i].nome, tof_cal_ganho(i), tof_cal_offset(i),
                  tof_cal_gravada(i) ? "cal (ESP32)" : "config.h (sem cal)");
}

// Comando calzera: remoção da calibração de um sensor ou de todos.
static void calzera(const char* arg) {
  if (!strcasecmp(arg, "todos")) { for (uint8_t i = 0; i < N_TOF; i++) tof_cal_zera(i); Log.println(F("# calibracoes apagadas")); return; }
  int8_t i = tof_indice(arg);
  if (i < 0) { Log.println(F("! uso: calzera FC   ou   calzera todos")); return; }
  tof_cal_zera(i);
  Log.printf("# calibracao de %s apagada\n", TOF[i].nome);
}

// Comando medir: o sensor mede 50 vezes com o alvo a uma distância conhecida e o erro aparece no terminal.
static void medir(const char* args) {
  char nome[8] = {0};
  int mm = 0;
  if (sscanf(args, "%7s %d", nome, &mm) != 2 || mm <= 0) { Log.println(F("! uso: medir FC 200")); return; }
  int8_t i = sensorDoArg(nome);
  if (i < 0) return;
  float med, dp;
  Log.printf("# medindo %s com alvo a %d mm...\n", TOF[i].nome, mm);
  if (!mediaBruta(i, 50, med, dp)) { Log.println(F("! poucas leituras validas: alvo dentro do cone e do alcance?")); return; }
  float corr = tof_cal_ganho(i) * med + tof_cal_offset(i);
  Log.printf("# %s: bruto %.1f | corrigido %.1f mm | erro %+.1f mm | ruido +-%.1f mm\n",
                TOF[i].nome, med, corr, corr - mm, dp * tof_cal_ganho(i));
}

// Modos contínuos, chamados pelo debug_passo() de tempos em tempos.
// Modo ver: uma linha com todos os sensores, o oponente, a borda, o rumo e o estado.
static void imprimeVer() {
  for (uint8_t i = 0; i < N_TOF; i++) {
    int16_t v = tof_mm(i);
    if (v < 0) Log.printf("%s: ----  ", TOF[i].nome);
    else       Log.printf("%s:%4d  ", TOF[i].nome, v);
  }
  if (P.op.visto) Log.printf("| OP %+6.1f (+-%4.1f) %4dmm conf %3.0f%% %s", P.op.angulo, P.op.sigma, P.op.dist_mm,
                             P.op.confianca * 100, P.op.centrado ? "CENTRO" : "      ");
  else if (P.op.existe < 0.3f) Log.print("| OP  -- arena vazia            ");
  else            Log.printf("| OP  -- provavel %s (%2.0f%%)    ", P.op.lado_ultimo > 0 ? "ESQ" : P.op.lado_ultimo < 0 ? "DIR" : "?  ", P.op.existe * 100);
#if USAR_BORDA
  Log.printf(" | BORDA %s", P.borda.detectada ? (P.borda.lado < 0 ? "ESQ" : P.borda.lado > 0 ? "DIR" : "AMBAS") : "-");
#endif
  static const char* RM[] = {"-", "ATACAR", "MIRAR", "PROVAVEL", "PROCURAR"};   // mesma ordem do RumoModo
  Log.printf(" | rumo %s %+4.0f", RM[P.rumo.modo < 5 ? P.rumo.modo : 0], P.rumo.angulo);
  Log.printf(" | %-10s | mot %4d %4d | ToF %u/%u\n", estado_nome(estado_atual()), motores_esq(), motores_dir(),
                P.saude.tof_ok, P.saude.tof_total);
}

// Modo id: aviso quando um sensor acabou de ser coberto (leitura abaixo de 60 mm), para conferir
// se a etiqueta física bate com o nome da tabela. Só a mudança de descoberto para coberto é avisada.
static void imprimeId() {
  uint16_t coberto = 0;
  for (uint8_t i = 0; i < N_TOF; i++) { int16_t r = tof_bruto(i); if (r > 0 && r < 60) coberto |= (1u << i); }
  for (uint8_t i = 0; i < N_TOF; i++)
    if ((coberto & (1u << i)) && !(cobertoAnt & (1u << i)))
      Log.printf(">> COBERTO: %s  (XSHUT no GPIO%d). Confere com a etiqueta?\n", TOF[i].nome, TOF[i].xshut);
  cobertoAnt = coberto;
}

// Modo borda: leitura em mV, limiar e situação de cada TCRT5000.
static void imprimeBorda() {
  for (uint8_t i = 0; i < N_BORDA; i++)
    Log.printf("%s:%4umV(lim %4u) %s  ", BORDA[i].nome, borda_mv(i), borda_limiar(i),
               (P.borda.mascara & (1u << i)) ? "BORDA!" : borda_branco_cru(i) ? "branco" : "preto ");
  Log.println();
}

// Calibração da borda.
// Média e desvio de n leituras (mV) de todos os sensores de borda ao mesmo tempo.
static void __attribute__((unused)) mediaBorda(uint16_t n, float* med, float* dp) {
  float s[N_BORDA] = {0}, s2[N_BORDA] = {0};
  for (uint16_t k = 0; k < n; k++) {
    servico();
    for (uint8_t i = 0; i < N_BORDA; i++) { float v = borda_mv(i); s[i] += v; s2[i] += v * v; }
    delay(2);
  }
  for (uint8_t i = 0; i < N_BORDA; i++) { med[i] = s[i] / n; dp[i] = sqrtf(max(0.0f, s2[i] / n - med[i] * med[i])); }
}

// Diagnóstico e calibração da borda em 3 medidas: ar (nada embaixo), preto e branco.
// A medida no ar mostra se chega luz no receptor sem chão embaixo, o que indica reflexo em uma
// superfície branca colada no sensor (a protoboard, por exemplo) ou luz forte do ambiente.
// O preto e o branco dão o contraste e a polaridade, já que alguns módulos são invertidos.
// O trimpot azul do módulo não muda nada aqui, porque ele só ajusta a saída D0 e o código lê a A0.
// O branco é opcional: sem ele, o limiar é estimado a partir do preto.
static void calBorda() {
#if USAR_BORDA
  if (estado_atual() != ESPERA) { Log.println(F("! pare o robo antes (stop)")); return; }
  Log.println(F("# ===== BORDA: DIAGNOSTICO + CALIBRACAO (3 medidas) ====="));
  Log.println(F("# O trimpot azul NAO importa: ele so mexe na saida D0, e o codigo le a A0."));
  float ar[N_BORDA], pr[N_BORDA], br[N_BORDA], dp[N_BORDA], med[N_BORDA];
  bool temBr[N_BORDA];

  Log.println(F("> 1/3 AR: levante o robo uns 10 cm, sem nada embaixo dos sensores (nem mao). ENTER mede (x cancela)."));
  if (!esperaEnter()) { Log.println(F("! cancelado, nada foi gravado")); return; }
  mediaBorda(60, ar, dp);
  for (uint8_t i = 0; i < N_BORDA; i++) Log.printf("# %s no ar: %.0f mV (+-%.0f)\n", BORDA[i].nome, ar[i], dp[i]);

  Log.println(F("> 2/3 PRETO: apoie o robo no preto, os 4 sensores no preto. ENTER mede (x cancela)."));
  if (!esperaEnter()) { Log.println(F("! cancelado, nada foi gravado")); return; }
  mediaBorda(100, pr, dp);
  for (uint8_t i = 0; i < N_BORDA; i++) Log.printf("# %s no preto: %.0f mV (+-%.0f)\n", BORDA[i].nome, pr[i], dp[i]);

  for (uint8_t i = 0; i < N_BORDA; i++) {
    Log.printf("> 3/3 BRANCO: so o sensor %s em cima do branco (faixa do dohyo ou papel sulfite). ENTER mede, x pula.\n", BORDA[i].nome);
    temBr[i] = esperaEnter();
    if (temBr[i]) { mediaBorda(60, med, dp); br[i] = med[i]; Log.printf("# %s no branco: %.0f mV (+-%.0f)\n", BORDA[i].nome, br[i], dp[i]); }
  }

  Log.println(F("# ----- resultado -----"));
  uint8_t gravados = 0;
  // O laço analisa cada sensor: descobre a polaridade, calcula quanta luz chega no ar e no
  // preto, mostra os avisos de montagem e grava a calibração quando não tem problema.
  for (uint8_t i = 0; i < N_BORDA; i++) {
    const char* n = BORDA[i].nome;
    // polaridade: mais luz dá tensão menor (módulo comum) ou maior (módulo invertido)
    bool abaixo;
    if (temBr[i] && fabsf(pr[i] - br[i]) >= 100) abaixo = br[i] < pr[i];
    else if (fabsf(pr[i] - ar[i]) >= 150)        abaixo = pr[i] < ar[i];      // preto reflete um pouco mais que o ar
    else if (ar[i] > 1650)                       abaixo = true;               // escuro = tensão alta (módulo comum)
    else if (ar[i] < 500)                        abaixo = false;              // escuro = tensão baixa (invertido)
    else                                         abaixo = BORDA_BRANCO_ABAIXO;
    // "luz" vai de 0 (escuro total do módulo) a 1 (branco)
    float escuro = abaixo ? 3300.0f : 0.0f;
    // Sem o branco medido, o código estima um branco que coloca o limiar na metade da leitura do
    // preto (na bancada: preto 1070 mV e branco 259 mV deram limiar de 542 mV, perto da metade).
    float alvoBr = temBr[i] ? br[i] : (abaixo ? 0.2308f * pr[i] : (0.15f * pr[i] + 1650.0f) / 0.65f);
    float faixa = fabsf(escuro - alvoBr);
    if (faixa < 1) faixa = 1;
    float luzAr = fabsf(escuro - ar[i]) / faixa, luzPreto = fabsf(escuro - pr[i]) / faixa;
    float contraste = temBr[i] ? fabsf(pr[i] - br[i]) : NAN;
    char tb[8] = "-";
    if (temBr[i]) snprintf(tb, sizeof(tb), "%d", (int)lroundf(br[i]));
    Log.printf("# %s: ar %.0f | preto %.0f | branco %s | luz no ar %.0f%%, no preto %.0f%% | %s\n", n, ar[i], pr[i],
               tb, luzAr * 100, luzPreto * 100,
               abaixo ? "branco = tensao BAIXA" : "branco = tensao ALTA (modulo invertido, o codigo se ajusta)");
    bool ok = true;
    if (luzAr > 0.25f) {
      Log.printf("! %s: chega luz no receptor SEM chao embaixo. Causa provavel: a lateral BRANCA da protoboard\n", n);
      Log.println(F("!    colada no sensor reflete o infravermelho direto no receptor (ou ha sol/lampada forte)."));
      Log.println(F("!    Conserto: cubra a lateral branca perto do sensor com 2-3 voltas de fita isolante PRETA,"));
      Log.println(F("!    ou afaste o sensor uns 5 mm com um calco preto. A face do sensor deve ser o ponto mais baixo."));
    }
    if (temBr[i] && contraste < 300) {
      if (luzAr <= 0.25f && fabsf(pr[i] - ar[i]) < 150 && fabsf(br[i] - ar[i]) < 150)
        Log.printf("! %s: a leitura quase nao muda (ar = preto = branco). Ou o receptor esta cego de luz (reflexo da\n"
                   "!    protoboard branca: tape o sensor com o dedo e veja se muda), ou falta VCC, GND ou o fio A0 -> GPIO%d.\n", n, BORDA[i].pino);
      else if (luzAr <= 0.25f)
        Log.printf("! %s: o seu PRETO reflete infravermelho quase como o branco. Teste em fita isolante preta ou no dohyo.\n", n);
      ok = false;
    }
    if (!temBr[i] && fabsf(pr[i] - ar[i]) < 100 && luzAr > 0.25f)
      Log.printf("! %s: no ar e no preto a leitura e quase a mesma. Meca o branco para ter certeza.\n", n);
    if (ok) {
      borda_cal_define(i, (uint16_t)lroundf(pr[i]), (uint16_t)lroundf(alvoBr));
      gravados++;
      Log.printf("# %s OK: limiar %u mV%s (gravado)\n", n, borda_limiar(i), temBr[i] ? "" : " (branco ESTIMADO: meca o branco quando puder)");
    } else Log.printf("# %s NAO gravado: resolva o problema acima e rode calborda de novo.\n", n);
  }
  Log.printf("# %u de %u sensores de borda calibrados. Passe o robo sobre a faixa branca: o painel mostra o aviso.\n",
             gravados, (unsigned)N_BORDA);
#else
  Log.println(F("! USAR_BORDA esta 0 em config.h"));
#endif
}

// Protocolo de texto entre o ESP32 e o painel (painel_sumo.html).
// Cada linha começa com uma letra que indica o tipo. O cabeçalho é enviado nos comandos s1 e geo
// (o painel manda s1 logo depois de abrir a porta USB) e depois de cada calibração:
//   G,i,nome,x,y,ang            geometria de cada ToF
//   I,i,ok                      sensor respondendo
//   C,i,ganho,offset,gravada    calibração do ToF
//   R,i,nome,x,y,altura,limiar,preto,branco,gravada,abaixo    sensores de borda
//   B,frente,tras,esq,dir,alt_corpo,alt_lente,nomeGeometria  corpo do robô
//   E,nome0,nome1,...           nomes dos estados
//   K,alcance,map...            mantido para ser compatível com o radar antigo
//   X,bayes,ladoOponente,alcancePreto,massaMin     parâmetros do rastreador
// Dados enviados a cada STREAM_MS:
//   D,mm0..mmN,bruto0..brutoN,visto,ang*10,dist,conf,mascara,cx,cy,acao,lado,estado,saude,millis,
//     sigma*10,confianca*100,existe*100,rumoModo,rumoAng*10,buscaGanho*100
//     (cx e cy são o centro do oponente; rumoModo: 0 nada, 1 atacar, 2 mirar, 3 provável, 4 procurar)
//   Q,bordaConfirmada,bordaCru,mv0..mv3
//   M,40,19,140,40,<mapa>       a cada MAPA_MS (formato explicado no rastreador.h)
static void streamCabecalho() {
  for (uint8_t i = 0; i < N_TOF; i++) {
    Log.printf("G,%u,%s,%d,%d,%d\n", i, TOF[i].nome, TOF[i].x, TOF[i].y, TOF[i].ang);
    Log.printf("I,%u,%u\n", i, tof_ok(i) ? 1 : 0);
    Log.printf("C,%u,%.4f,%.2f,%u\n", i, tof_cal_ganho(i), tof_cal_offset(i), tof_cal_gravada(i) ? 1 : 0);
  }
#if USAR_BORDA
  for (uint8_t i = 0; i < N_BORDA; i++)
    Log.printf("R,%u,%s,%d,%d,%.1f,%u,%u,%u,%u,%u\n", i, BORDA[i].nome, BORDA[i].x, BORDA[i].y, BORDA_ALTURA_MM,
               borda_limiar(i), borda_cal_preto(i), borda_cal_branco(i), borda_cal_gravada(i) ? 1 : 0, borda_branco_abaixo(i) ? 1 : 0);
#endif
  Log.printf("B,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%s\n", CORPO_FRENTE_MM, CORPO_TRAS_MM, CORPO_ESQ_MM, CORPO_DIR_MM,
                CORPO_ALTURA_MM, LENTE_ALTURA_MM, GEOMETRIA_NOME);
  Log.print("E");
  for (uint8_t e = 0; e < N_ESTADOS; e++) Log.printf(",%s", estado_nome((Estado)e));
  Log.println();
  Log.printf("K,%d", TOF_ALCANCE_MM);
  for (uint8_t i = 0; i < N_TOF; i++) Log.printf(",%u", i);
  Log.printf(",%d,%d,0,0\n", TOF_TIMING_US > 25000 ? 1 : 0, MODO_DEFENSIVO);
  Log.printf("X,%d,%d,%d,%.2f\n", oponente_bayes() ? 1 : 0, OPONENTE_LADO_MM, ALCANCE_PRETO_MM, FUSAO_MASSA_MIN);
}

// Linha M com o mapa de probabilidade comprimido pelo rastreador.cpp.
static void streamMapa() {
  static char m[MAPA_NB * MAPA_NR + 4];
  uint16_t n = rastreador_mapa(m, sizeof(m));
  if (!n) return;
  Log.printf("M,%d,%d,%d,%d,", MAPA_NB, MAPA_NR, MAPA_R0, MAPA_DR);
  Log.write((const uint8_t*)m, n);
  Log.println();
}

// Linhas D e Q com o estado atual de toda a percepção, desenhadas pelo painel.
static void streamDados() {
  Log.print('D');
  for (uint8_t i = 0; i < N_TOF; i++) Log.printf(",%d", tof_mm(i));
  for (uint8_t i = 0; i < N_TOF; i++) Log.printf(",%d", tof_bruto(i));
  uint8_t conf = __builtin_popcount(P.op.mascara);
  uint8_t acao = 0;                                   // 0 busca, 1 ataca, 2 e 3 curva, 4 e 5 giro, 6 segura
  if (P.op.visto) {
    if (P.op.centrado) acao = MODO_DEFENSIVO ? 6 : 1;
    else if (fabsf(P.op.angulo) <= 40) acao = P.op.angulo > 0 ? 2 : 3;
    else acao = P.op.angulo > 0 ? 4 : 5;
  }
  Log.printf(",%d,%d,%d,%u,%u,%d,%d,%u,%d,%u,%u,%lu,%d,%d,%d,%u,%d,%d\n", P.op.visto ? 1 : 0, (int)lroundf(P.op.angulo * 10),
                P.op.dist_mm, conf, P.op.mascara, (int)lroundf(P.op.x), (int)lroundf(P.op.y),
                acao, P.op.lado_ultimo, (unsigned)estado_atual(), P.saude.tof_mascara, (unsigned long)millis(),
                (int)lroundf(P.op.sigma * 10), (int)lroundf(P.op.confianca * 100), (int)lroundf(P.op.existe * 100),
                (unsigned)P.rumo.modo, (int)lroundf(P.rumo.angulo * 10), (int)lroundf(P.op.busca_ganho * 100));
#if USAR_BORDA
  // a borda vai em uma segunda linha (Q) para a linha D não passar do tamanho máximo
  Log.printf("Q,%u,%u", P.borda.mascara, P.borda.cru);
  for (uint8_t i = 0; i < N_BORDA; i++) Log.printf(",%u", borda_mv(i));
  Log.println();
#endif
}

// Interpretação de uma linha de comando. Uma cópia com as letras originais é guardada para os
// argumentos (nome do sensor), e a comparação dos comandos é feita em minúsculas.
static void executa(char* c) {
  while (*c == ' ') c++;                // espaços no começo são ignorados
  char original[48];
  strncpy(original, c, sizeof(original) - 1);
  original[sizeof(original) - 1] = 0;
  for (char* p = c; *p; p++) *p = tolower(*p);
  if      (!strcmp(c, "help") || !strcmp(c, "?")) ajuda();
  else if (!strcmp(c, "lista"))  lista();
  else if (!strcmp(c, "scan"))   scan();
  else if (!strcmp(c, "ver"))    { modoVer = !modoVer;     Log.printf("# ver %s\n", modoVer ? "ON" : "OFF"); }
  else if (!strcmp(c, "id"))     { modoId = !modoId; cobertoAnt = 0;
                                   Log.println(modoId ? F("# ID ON: cubra UM sensor por vez com a mao (2-5 cm)") : F("# ID OFF")); }
  else if (!strcmp(c, "borda"))  { modoBorda = !modoBorda; Log.printf("# borda %s\n", modoBorda ? "ON" : "OFF");
#if !USAR_BORDA
                                   Log.println(F("! USAR_BORDA esta 0 em config.h"));
#endif
                                 }
  else if (!strncmp(c, "cal ", 4))     { calibra(original + 4); streamCabecalho(); }
  else if (!strcmp(c, "calver"))       calver();
  else if (!strcmp(c, "calborda") || !strcmp(c, "diagborda")) { calBorda(); streamCabecalho(); }
  else if (!strcmp(c, "bordazera"))    { borda_cal_zera(); Log.println(F("# calibracao da borda apagada")); streamCabecalho(); }
  else if (!strncmp(c, "calzera ", 8)) { calzera(original + 8); streamCabecalho(); }
  else if (!strncmp(c, "medir ", 6))   medir(original + 6);
  else if (!strcmp(c, "init")) {
    if (estado_atual() != ESPERA) Log.println(F("! pare o robo antes (stop)"));
    else { uint8_t n = tof_reinicia_falhos(); Log.printf("# %u sensor(es) recuperado(s)\n", n); lista(); }
  }
  else if (!strcmp(c, "tempo")) {
    Log.printf("# ciclo atual %lu us | pior %lu us (%.0f Hz no pior caso)\n", (unsigned long)P.saude.ciclo_us,
                  (unsigned long)P.saude.ciclo_us_max, P.saude.ciclo_us_max ? 1e6 / P.saude.ciclo_us_max : 0.0);
    Log.printf("# fusao: %s", oponente_bayes() ? "rastreador bayesiano" : "simples");
    if (oponente_bayes()) Log.printf(" | passo %lu us, pior %lu us (a cada %d ms)", (unsigned long)rastreador_us(),
                                     (unsigned long)rastreador_us_max(), TOF_TIMING_US / 1000);
    Log.printf(" | linhas de dados descartadas na USB: %lu\n", (unsigned long)telemetria_descartes_usb());
    P.saude.ciclo_us_max = 0;
    rastreador_us_zera();
  }
  else if (!strcmp(c, "go"))     { estados_iniciar(); Log.println(F("# start: contagem de 5 s")); }
  else if (!strcmp(c, "stop"))   { estados_parar();   Log.println(F("# parado")); }
  else if (!strcmp(c, "s1") || !strcmp(c, "geo") || !strcmp(c, "cfg")) { modoStream = modoStream || !strcmp(c, "s1"); streamCabecalho(); }
  else if (!strcmp(c, "s0"))     modoStream = false;
  else Log.printf("! comando desconhecido: %s (digite help)\n", c);
}

void debug_init() { ajuda(); lista(); }   // no boot aparecem os comandos e a tabela de sensores

// O loop do sumo_esp32.ino chama esta função em toda volta. Primeiro o laço junta os caracteres
// até formar uma linha e executa o comando; depois os modos contínuos ligados são impressos,
// cada um no seu ritmo. O painel liga o stream sozinho, enviando s1 logo depois de conectar.
void debug_passo() {
  int lido;
  while ((lido = entrada_le()) >= 0) {
    char ch = (char)lido;
    if (ch == '\r' || ch == '\n') {
      if (nLinha) { linha[nLinha] = 0; nLinha = 0; executa(linha); }
    } else if (nLinha < sizeof(linha) - 1) linha[nLinha++] = ch;
  }
  uint32_t t = millis();
  if (modoVer    && t - tVer    >= 200) { tVer = t;    imprimeVer(); }
  if (modoId     && t - tId     >= 50)  { tId = t;     imprimeId(); }
  if (modoBorda  && t - tBorda  >= 200) { tBorda = t;  imprimeBorda(); }
  if (modoStream && t - tStream >= STREAM_MS) { tStream = t; streamDados(); }
  if (modoStream && oponente_bayes() && t - tMapa >= MAPA_MS) { tMapa = t; streamMapa(); }
}
