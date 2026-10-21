// debug.cpp
// Comandos de teste, calibração e o stream de dados para o painel. Os comandos chegam pela
// entrada_le() do telemetria.cpp (Monitor Serial pela USB, painel pela USB ou pelo WiFi) e o
// comando help mostra a lista completa.
// Este é o único módulo que conversa com todos os outros, porque precisa mostrar tudo.
#include "debug.h"
#include "config.h"
#include "telemetria.h"
#include "percepcao.h"
#include "oponente.h"
#include "borda.h"
#include "motores.h"
#include "cinematica.h"
#include "estados.h"
#include "rastreador.h"
#include "arena.h"
#include "chave.h"
#include "antitrava.h"   // ANTI_TRAVA
#include "ajustes.h"
#include <Wire.h>
#include <strings.h>   // strcasecmp

static char     linha[48];          // comando que está sendo digitado
static uint8_t  nLinha = 0;
// modos contínuos ligados e desligados pelos comandos, cada um com o horário da última impressão
static bool     modoVer = false, modoId = false, modoBorda = false, modoStream = false;
static uint32_t tVer = 0, tId = 0, tBorda = 0, tStream = 0, tMapa = 0;
static uint16_t cobertoAnt = 0;     // sensores cobertos na leitura anterior do modo id

static void streamCabecalho();
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
  Log.println(F("# calpista on|off    janelas de minimo/maximo da borda para a calibracao de pista do painel"));
  Log.println(F("# bordalim NOME LIM [PRETO BRANCO]  grava o limiar da borda (mV) calculado pelo painel"));
  Log.println(F("# init               tenta religar sensores que falharam (robo parado)"));
  Log.println(F("# tempo              duracao do ciclo (atual e pior caso)"));
  Log.println(F("# cin                modelo das rodas: velocidade, giro, feed-forward e parada"));
  Log.println(F("# mov V W MS [F]     teste parado: V mm/s e W graus/s por MS ms (ate 3000); F: 1 ignora a linha, 2 sem texto, 3 os dois"));
  Log.println(F("# pwm E D MS [F]     teste dos fios: E% e D% de PWM (-100 a 100) por MS ms; F como no mov"));
  Log.println(F("# alinha on [S]      demo: so gira em pulsos ate alinhar com o oponente (S segundos, padrao 60); alinha off para"));
  Log.println(F("# ajuste             opcoes do painel gravadas no robo: auto, linha, tof, alinha, wifi (ajuste NOME VALOR | ajuste padrao)"));
  Log.println(F("# estrategia rush|flank|fan|chave   escolhe a estrategia (chave = segue a DIP)"));
  Log.println(F("# perfil claro|escuro|chave         perfil do oponente (chave = segue a DIP 4)"));
  Log.println(F("# arena              localizacao no dojo: centro, incerteza e espaco livre"));
#if ANTI_TRAVA
  Log.println(F("# trava [N]          manobra do empurrao travado: 0 nenhuma 1 lateral 2 torque 3 zigue 4 giro 5 pulso"));
#endif
  Log.println(F("# go | stop          inicia (5 s) / para a maquina de estados"));
  Log.println(F("# s1 | s0            liga/desliga stream para o painel (HTML)"));
  Log.println(F("# wifi | wifi off    rede, endereco e paineis conectados / desliga o radio (USAR_WIFI 1)"));
  Log.println(F("# ============================================"));
}

// Comando lista: tabela de sensores do config.h junto com a calibração e o status de cada um.
static void lista() {
  Log.printf("# geometria: %s\n", GEOMETRIA_NOME);
  Log.println(F("# nome  XSHUT   end.   x    y   ang   ganho  offset  cal     status"));
  for (uint8_t i = 0; i < N_TOF; i++)
    Log.printf("# %-4s  GPIO%-2d  0x%02X %4d %4d %4d  %6.3f %+6.1f  %-6s  %s\n", TOF[i].nome, TOF[i].xshut,
                  TOF[i].endereco, TOF[i].x, TOF[i].y, TOF[i].ang, tof_cal_ganho(i), tof_cal_offset(i),
                  tof_cal_gravada(i) ? "ESP32" : "config",
                  !tof_ok(i) ? "FALHOU" : ((tof_obstruidos() >> i) & 1 ? "OBSTRUIDO (le menos de 15 mm parado)" : "OK"));
  Log.printf("# sensor central (decide ataque): %s\n", tof_centro() >= 0 ? TOF[tof_centro()].nome : "-");
  uint8_t b = chave_bits();
  Log.printf("# chave DIP: %d%d%d%d -> estrategia %s, oponente %s (sensores no perfil %s)%s\n", b & 1, (b >> 1) & 1,
             (b >> 2) & 1, (b >> 3) & 1, estrategia_nome(chave_estrategia()), chave_escuro() ? "ESCURO" : "CLARO",
             tof_perfil_escuro() ? "ESCURO" : "CLARO", chave_conflito() ? " | ! mais de uma estrategia ligada" : "");
  Log.printf("# %u de %u OK -> %s\n", P.saude.tof_ok, P.saude.tof_total,
                P.saude.pronto ? "PRONTO para lutar" : "NAO inicia (minimo TOF_MINIMO_PARA_LUTAR)");
#if USAR_BORDA
  Log.println(F("# borda  GPIO   x    y   limiar  preto  branco  cal"));
  for (uint8_t i = 0; i < N_BORDA; i++)
    Log.printf("# %-4s   %2d  %4d %4d  %5u  %5u  %5u   %s\n", BORDA[i].nome, BORDA[i].pino, borda_x(i), borda_y(i),
               borda_limiar(i), borda_cal_preto(i), borda_cal_branco(i),
               borda_cal_gravada(i) ? (borda_cal_limiar_gravado(i) ? "pista" : "ESP32") : "padrao");
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
  tof_i2c_pega();                       // o I2C fica reservado enquanto a varredura roda
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
  tof_i2c_solta();
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
  if (P.arena.valida) Log.printf(" | centro %+4.0f %4.0fmm (+-%3.0f)%s", P.arena.ang, P.arena.dist, P.arena.sigma, P.arena.confiavel ? "" : "?");
  Log.printf(" | %-10s %s | mot %4d %4d | rodas %+4.0f %+4.0f mm/s | ToF %u/%u\n", estado_nome(estado_atual()),
                estrategia_nome(estados_estrategia()), motores_esq(), motores_dir(), cinematica_roda_esq(), cinematica_roda_dir(),
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
// O trimpot azul do módulo só ajusta a saída D0. Com o fio no A0 ele não muda nada; com o fio no D0
// o sinal é digital (cerca de 0 V no branco e 3,3 V no preto), o trimpot decide a borda e esta
// calibração não refina nada (o aviso aparece no resultado).
// O branco é opcional: sem ele, o limiar é estimado a partir do preto.
static void calBorda() {
#if USAR_BORDA
  if (estado_atual() != ESPERA) { Log.println(F("! pare o robo antes (stop)")); return; }
  Log.println(F("# ===== BORDA: DIAGNOSTICO + CALIBRACAO (3 medidas) ====="));
  Log.println(F("# O trimpot azul so mexe na saida D0: com o fio no A0 ele NAO importa; com o fio no D0 o sinal e digital e o trimpot decide."));
  Log.println(F("# Para um limiar que valha na pista inteira (arranhoes, volta pela faixa), use a calibracao de pista do painel."));
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
    // sinal digital: o fio está no D0 do módulo (saída do comparador), que só vale 0 V ou 3,3 V
    bool digital = (ar[i] < 150 || ar[i] > 3000) && (pr[i] < 150 || pr[i] > 3000) && (!temBr[i] || br[i] < 150 || br[i] > 3000);
    if (digital)
      Log.printf("! %s: so le 0 V ou 3,3 V (sinal DIGITAL). O fio esta no D0 do modulo? Quem decide a borda e o trimpot azul; para o limiar do firmware valer, ligue o fio no A0.\n", n);
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

// Índice do sensor de borda pelo nome (BFE, BFD, BTE, BTD), sem diferenciar maiúscula de minúscula; -1 se não existir.
static int8_t bordaIndice(const char* nome) {
  for (uint8_t i = 0; i < N_BORDA; i++) if (!strcasecmp(nome, BORDA[i].nome)) return (int8_t)i;
  return -1;
}

// Comando calpista on|off: liga as janelas de mínimo, máximo e média dos sensores de borda (linha J do
// protocolo), usadas pela calibração de pista do painel. Sem argumento, mostra o estado.
static void calPista(const char* arg) {
#if USAR_BORDA
  if (!strcmp(arg, "on") || !strcmp(arg, "1")) {
    if (estado_atual() != ESPERA) { Log.println(F("! pare o robo antes (stop)")); return; }
    borda_pista(true);
    Log.println(F("# calpista ON: linhas J com minimo, maximo e media de cada sensor de borda (desliga sozinho ao sair da ESPERA)"));
  } else if (!strcmp(arg, "off") || !strcmp(arg, "0")) {
    borda_pista(false);
    Log.println(F("# calpista OFF"));
  } else Log.printf("# calpista %s (uso: calpista on|off)\n", borda_pista_ativa() ? "ON" : "OFF");
#else
  (void)arg;
  Log.println(F("! USAR_BORDA esta 0 em config.h"));
#endif
}

// Comando bordalim NOME LIM [PRETO BRANCO]: grava o limiar (mV) de um sensor de borda, calculado pelo painel com
// o pior preto e o pior branco da pista. Preto e branco são os valores típicos (padrão: 3 vezes e 0,3 vez o limiar).
static void bordaLim(const char* arg) {
#if USAR_BORDA
  char nome[8] = "";
  int lim = 0, p = 0, b = 0;
  int n = sscanf(arg, "%7s %d %d %d", nome, &lim, &p, &b);
  int8_t i = n >= 2 ? bordaIndice(nome) : -1;
  if (i < 0 || lim < 50 || lim > 3300 || (n == 3)) {
    Log.println(F("! uso: bordalim NOME LIM [PRETO BRANCO]   (NOME: BFE, BFD, BTE ou BTD; LIM de 50 a 3300 mV)"));
    return;
  }
  if (estado_atual() != ESPERA) { Log.println(F("! pare o robo antes (stop)")); return; }
  if (n < 4) { p = lim * 3; b = lim / 3; }
  p = constrain(p, 0, 4095); b = constrain(b, 0, 4095);
  if (p == b) { Log.println(F("! preto e branco iguais: o sentido do sensor ficaria indefinido")); return; }
  borda_cal_limiar((uint8_t)i, (uint16_t)lim, (uint16_t)p, (uint16_t)b);
  Log.printf("# %s: limiar de pista %u mV gravado (preto %u, branco %u, branco = tensao %s)\n", BORDA[i].nome, borda_limiar(i),
             borda_cal_preto(i), borda_cal_branco(i), borda_branco_abaixo(i) ? "BAIXA" : "ALTA");
  streamCabecalho();
#else
  (void)arg;
  Log.println(F("! USAR_BORDA esta 0 em config.h"));
#endif
}

// Protocolo de texto entre o ESP32 e o painel (painel_sumo.html).
// Cada linha começa com uma letra que indica o tipo. O cabeçalho é enviado nos comandos s1 e geo
// (o painel manda s1 logo depois de abrir a porta USB) e depois de cada calibração:
//   G,i,nome,x,y,ang            geometria de cada ToF
//   I,i,ok                      sensor respondendo
//   C,i,ganho,offset,gravada    calibração do ToF
//   R,i,nome,x,y,altura,limiar,preto,branco,gravada,abaixo,limiarDePista    sensores de borda
//   B,frente,tras,esq,dir,alt_corpo,alt_lente,nomeGeometria  corpo do robô
//   E,nome0,nome1,...           nomes dos estados
//   K,alcance,map...            mantido para ser compatível com o radar antigo
//   X,bayes,ladoOponente,alcancePreto,massaMin     parâmetros do rastreador
//   F,usarMotores,usarWifi,autoStart,usarBorda,qtdTof,bancada,wifiLigado,linhaLateral,alinhaContagem,wifiLuta
//     (chaves do config.h compiladas no firmware e, de autoStart em diante, as opções do painel gravadas na NVS: ajustes.h)
// Dados enviados a cada STREAM_MS:
//   D,mm0..mmN,bruto0..brutoN,visto,ang*10,dist,conf,mascara,cx,cy,acao,lado,estado,saude,millis,
//     sigma*10,confianca*100,existe*100,rumoModo,rumoAng*10,buscaGanho*100
//     (cx e cy são o centro do oponente; rumoModo: 0 nada, 1 atacar, 2 mirar, 3 provável, 4 procurar)
//   Q,bordaConfirmada,bordaCru,mv0..mv3
//   Z,fase,erro*10,segundos,pulsos   demo de alinhamento: fase 0 desligado, 1 buscando, 2 virando, 3 alinhado
//     (erro é o ângulo até o oponente, positivo à esquerda; só com o comando alinha ligado)
//   J,seq,voltas,min0,max0,med0,min1,max1,med1,...   janela dos sensores de borda em mV, só com calpista ligado
//     (seq conta as janelas enviadas, então um salto mostra janela perdida; voltas é quantas leituras entraram)
//   A,estrategia,escuro,dip,arenaValida,arenaConfiavel,cx,cy,sigma,aproximacao,contato,descartes,
//     travado,destravas,aquecimento (travado: 0 livre, 1 empurrao travado, 2 giro preso, 3 manobra do
//     empurrao travado; aquecimento em K)
//     (estrategia: 0 Rush, 1 Flank, 2 Fan out; cx e cy: centro do dojô no referencial do robô, mm)
//   M,40,19,140,40,<mapa>       a cada MAPA_MS (formato explicado no rastreador.h)
static void streamCabecalho() {
  for (uint8_t i = 0; i < N_TOF; i++) {
    Log.printf("G,%u,%s,%d,%d,%d\n", i, TOF[i].nome, TOF[i].x, TOF[i].y, TOF[i].ang);
    Log.printf("I,%u,%u\n", i, tof_ok(i) ? 1 : 0);
    Log.printf("C,%u,%.4f,%.2f,%u\n", i, tof_cal_ganho(i), tof_cal_offset(i), tof_cal_gravada(i) ? 1 : 0);
  }
#if USAR_BORDA
  for (uint8_t i = 0; i < N_BORDA; i++)
    Log.printf("R,%u,%s,%d,%d,%.1f,%u,%u,%u,%u,%u,%u\n", i, BORDA[i].nome, borda_x(i), borda_y(i), BORDA_ALTURA_MM,
               borda_limiar(i), borda_cal_preto(i), borda_cal_branco(i), borda_cal_gravada(i) ? 1 : 0, borda_branco_abaixo(i) ? 1 : 0,
               borda_cal_limiar_gravado(i) ? 1 : 0);
#endif
  Log.printf("B,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%s\n", CORPO_FRENTE_MM, CORPO_TRAS_MM, CORPO_ESQ_MM, CORPO_DIR_MM,
                CORPO_ALTURA_MM, LENTE_ALTURA_MM, GEOMETRIA_NOME);
  Log.print("E");
  for (uint8_t e = 0; e < N_ESTADOS; e++) Log.printf(",%s", estado_nome((Estado)e));
  Log.println();
  Log.printf("K,%d", TOF_ALCANCE_MM);
  for (uint8_t i = 0; i < N_TOF; i++) Log.printf(",%u", i);
  Log.printf(",%d,%d,0,0\n", TOF_TIMING_US > 25000 ? 1 : 0, MODO_DEFENSIVO);
  Log.printf("X,%d,%d,%d,%.2f,%d,%d\n", oponente_bayes() ? 1 : 0, OPONENTE_LADO_MM, ALCANCE_PRETO_MM, FUSAO_MASSA_MIN,
                TOF_ALCANCE_LATERAL_MM, TOF_ALCANCE_TRAS_MM);
  const Ajustes& aj = ajustes();
  Log.printf("F,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", USAR_MOTORES, USAR_WIFI, aj.autonomo ? 1 : 0, USAR_BORDA, (int)aj.tof, GEOMETRIA_BANCADA,
             telemetria_wifi_ligado() ? 1 : 0, aj.borda_lateral ? 1 : 0, aj.alinha_contagem ? 1 : 0, aj.wifi_luta ? 1 : 0);
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
  // calibração de pista: a janela de mínimo, máximo e média desde o último envio. Fora da ESPERA
  // (luta) o modo desliga sozinho, para a linha extra não ocupar a serial durante a luta.
  if (borda_pista_ativa() && estado_atual() != ESPERA) { borda_pista(false); Log.println(F("# calpista desligado: o robo saiu da ESPERA")); }
  if (borda_pista_ativa()) {
    static uint32_t seqJanela = 0;
    uint16_t mn[N_BORDA], mx[N_BORDA], md[N_BORDA];
    uint16_t voltas = borda_pista_janela(mn, mx, md);
    if (voltas) {
      Log.printf("J,%lu,%u", (unsigned long)seqJanela++, voltas);
      for (uint8_t i = 0; i < N_BORDA; i++) Log.printf(",%u,%u,%u", mn[i], mx[i], md[i]);
      Log.println();
    }
  }
#endif
  bool ativa = estado_atual() != ESPERA;
  // demo de alinhamento (comando alinha): fase (0 desligado, 1 buscando, 2 virando, 3 alinhado), erro em décimos de grau,
  // segundos restantes e pulsos; a cada 100 ms e quando a fase muda, e uma vez com fase 0 depois de desligar
  {
    static uint8_t faseAntZ = 0;
    static uint32_t tZ = 0;
    uint8_t fz = estados_alinha_fase();
    uint32_t agora = millis();
    if ((fz || faseAntZ) && (fz != faseAntZ || agora - tZ >= 100)) {
      tZ = agora;
      Log.printf("Z,%u,%d,%u,%u\n", (unsigned)fz, (int)lroundf(estados_alinha_erro() * 10), (unsigned)estados_alinha_restante_s(),
                 (unsigned)estados_alinha_pulsos());
    }
    faseAntZ = fz;
  }
  Log.printf("A,%u,%u,%u,%u,%u,%d,%d,%d,%d,%u,%u,%u,%u,%d\n", (unsigned)(ativa ? estados_estrategia() : chave_estrategia()),
             (ativa ? estados_escuro() : chave_escuro()) ? 1 : 0, chave_bits(), P.arena.valida ? 1 : 0, P.arena.confiavel ? 1 : 0,
             (int)lroundf(P.arena.cx), (int)lroundf(P.arena.cy), (int)lroundf(P.arena.sigma), (int)lroundf(P.op.aproximacao),
             P.op.contato ? 1 : 0, tof_descartes(), estados_travado(), estados_destravas(),
             (int)lroundf(cinematica_aquecimento_k()));
}

// Comando arena: localização no dojô e espaço livre em 8 direções.
static void arena() {
  if (!P.arena.valida) { Log.println(F("# arena: a localizacao comeca no start (go)")); return; }
  Log.printf("# centro do dojo a %.0f mm, %+.0f graus (incerteza %.0f mm, %s) | leituras de fora descartadas: %u\n",
             P.arena.dist, P.arena.ang, P.arena.sigma, P.arena.confiavel ? "confiavel" : "ainda incerta", tof_descartes());
  Log.print("# espaco ate a borda:");
  for (int a = 0; a < 360; a += 45) Log.printf(" %+d:%.0f", a > 180 ? a - 360 : a, arena_livre(a > 180 ? a - 360 : a));
  Log.println(" mm");
  Log.printf("# passo do filtro: %lu us, pior %lu us desde o start (%d particulas a cada %d ms, no nucleo %d)\n",
             (unsigned long)arena_us(), (unsigned long)arena_us_max(), ARENA_N, ARENA_PASSO_MS, xPortGetCoreID());
}

// Comando cin: números do modelo das rodas (cinematica.h), calculados a partir do config.h.
// Servem para conferir o config e para comparar com as medidas do robô montado.
static void cin() {
  Log.printf("# motor: kS %.2f V | kV %.2f mV por mm/s | kA %.2f mV por mm/s2 | tensao no motor %.1f V\n",
                CIN_KS, CIN_KV * 1000, CIN_KA * 1000, CIN_V_DISP);
  Log.printf("# maximos: %.0f mm/s em linha reta | %.0f graus/s no eixo | 180 graus em %.0f ms\n",
                VEL_MAX_MMS, GIRO_MAX_DPS, 180.0f / GIRO_MAX_DPS * 1000);
  Log.printf("# borda: parada no pior caso %.1f mm | margem ate as rodas %.0f mm\n",
                DIST_PARADA_MM, BORDA_FAIXA_MM + BORDA_ATE_RODA_MM);
  Log.printf("# estimativa agora: %+.0f mm/s, %+.0f graus/s (motores %s)\n", cinematica_v_mms(), cinematica_w_dps(),
                USAR_MOTORES ? "ligados" : "desligados");
  Log.printf("# aquecimento estimado dos motores: %.0f K acima do ambiente (limite de %.0f%% do PWM a partir de %.0f K)%s\n",
                cinematica_aquecimento_k(), MOTOR_PWM_QUENTE * 100, MOTOR_QUENTE_K,
                cinematica_limite_termico() ? " | LIMITE LIGADO" : "");
}

// Opções do painel gravadas na NVS (ajustes.h). Só mudam com o robô parado. Mudar a quantidade de ToF reinicia o robô.
static uint32_t tReinicia = 0;       // diferente de zero: reinicia o ESP32 nesse instante (depois de gravar a quantidade de ToF)

static void ajustesImprime() {
  const Ajustes& a = ajustes();
  Log.printf("# ajustes (gravados no robo): auto %d | linha %d (%s) | tof %u | alinha %d | wifi %d%s\n", a.autonomo ? 1 : 0,
             a.borda_lateral ? 1 : 0, a.borda_lateral ? "colados de lado" : "como no CAD", (unsigned)a.tof, a.alinha_contagem ? 1 : 0,
             a.wifi_luta ? 1 : 0, ajustes_gravado() ? "" : " | valores de fabrica");
}

static void ajuste(const char* arg) {
  while (*arg == ' ') arg++;
  if (!*arg) { ajustesImprime(); return; }
  if (estado_atual() != ESPERA) { Log.println(F("! pare o robo antes (stop): as opcoes so mudam em ESPERA")); return; }
  int8_t r = AJUSTE_OK;
  char nome[10] = ""; int v = -1;
  if (!strcmp(arg, "padrao")) { r = ajustes_padrao(); Log.println(F("# ajustes: valores de fabrica")); }
  else {
    if (sscanf(arg, "%9s %d", nome, &v) != 2 || (r = ajustes_define(nome, v)) == AJUSTE_INVALIDO) {
      Log.println(F("! uso: ajuste auto|linha|alinha|wifi 0|1 | ajuste tof 5|7 | ajuste padrao | ajuste (mostra)"));
      return;
    }
    if (!strcmp(nome, "auto"))
      Log.println(v ? F("# modo autonomo gravado: vale quando o robo for ligado de novo (para largar agora, use go)")
                    : F("# modo autonomo desligado: o robo fica em ESPERA ate mandarem"));
  }
  if (r == AJUSTE_REINICIA) {
    Log.println(F("# tof gravado: o robo reinicia em 1 s para configurar os sensores"));
    tReinicia = millis() + 1000;
  } else ajustesImprime();
  streamCabecalho();                  // F e R com as opcoes novas (o painel redesenha os sensores de linha)
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
  else if (!strcmp(c, "calpista") || !strncmp(c, "calpista ", 9)) calPista(c + (c[8] ? 9 : 8));
  else if (!strncmp(c, "bordalim ", 9)) bordaLim(original + 9);
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
    if (oponente_bayes()) Log.printf(" | passo %lu us, pior %lu us (a cada %d ms, no nucleo %d)", (unsigned long)rastreador_us(),
                                     (unsigned long)rastreador_us_max(), (tof_perfil_escuro() ? ESCURO_TIMING_US : TOF_TIMING_US) / 1000,
                                     SENSORES_NUCLEO);
    Log.printf(" | linhas de dados descartadas na USB: %lu", (unsigned long)telemetria_descartes_usb());
#if USAR_WIFI
    Log.printf(" | no WiFi: %lu", (unsigned long)telemetria_descartes_wifi());
#endif
    Log.println();
    P.saude.ciclo_us_max = 0;
    rastreador_us_zera();
  }
  else if (!strcmp(c, "cin"))    cin();
  else if (!strcmp(c, "arena"))  arena();
  else if (!strncmp(c, "estrategia ", 11) || !strncmp(c, "perfil ", 7)) {
    bool estr = c[0] == 'e';
    const char* v = c + (estr ? 11 : 7);
    if (estado_atual() != ESPERA) Log.println(F("! pare o robo antes (stop): a escolha fica travada durante a luta"));
    else if (estr) {
      int8_t e = !strcmp(v, "rush") ? RUSH : !strcmp(v, "flank") ? FLANK : !strncmp(v, "fan", 3) ? FAN_OUT : -1;
      if (e < 0 && strcmp(v, "chave")) Log.println(F("! uso: estrategia rush|flank|fan|chave"));
      else { chave_forca_estrategia(e); Log.printf("# estrategia: %s%s\n", estrategia_nome(chave_estrategia()), e < 0 ? " (chave DIP)" : " (comando)"); }
    } else {
      int8_t e = !strcmp(v, "claro") ? 0 : !strcmp(v, "escuro") ? 1 : -1;
      if (e < 0 && strcmp(v, "chave")) Log.println(F("! uso: perfil claro|escuro|chave"));
      else { chave_forca_escuro(e); Log.printf("# perfil do oponente: %s%s\n", chave_escuro() ? "ESCURO" : "CLARO", e < 0 ? " (chave DIP 4)" : " (comando)"); }
    }
  }
  else if (!strncmp(c, "mov ", 4)) {
    float v = 0, w = 0; int ms = 0, fl = 0;
    int n = sscanf(c + 4, "%f %f %d %d", &v, &w, &ms, &fl);
    if (n >= 3 && ms > 0) estados_teste_mov(v, w, (uint16_t)(ms > 3000 ? 3000 : ms), (uint8_t)(fl & 3));
    else Log.println(F("! uso: mov V W MS [F]   (ex.: mov 300 0 1000 ou mov 0 360 1000; F: 1 ignora a linha, 2 sem texto)"));
  }
  else if (!strncmp(c, "pwm ", 4)) {
    int e = 0, d = 0, ms = 0, fl = 0;
    int n = sscanf(c + 4, "%d %d %d %d", &e, &d, &ms, &fl);
    if (n >= 3 && ms > 0)
      estados_teste_pwm((int8_t)constrain(e, -100, 100), (int8_t)constrain(d, -100, 100), (uint16_t)(ms > 3000 ? 3000 : ms), (uint8_t)(fl & 3));
    else Log.println(F("! uso: pwm E D MS [F]   (ex.: pwm 30 0 1000 so a roda esquerda para a frente; F como no mov)"));
  }
  else if (!strcmp(c, "alinha") || !strncmp(c, "alinha ", 7)) {
    char acao[8] = ""; int seg = 0;
    int n = c[6] == ' ' ? sscanf(c + 7, "%7s %d", acao, &seg) : 0;
    if (n >= 1 && (!strcmp(acao, "on") || !strcmp(acao, "1"))) {
#if !USAR_MOTORES
      Log.println(F("! USAR_MOTORES esta 0 em config.h: o alinha nao move as rodas. Ponha 1 e grave de novo."));
#endif
      estados_alinha(true, (uint16_t)constrain(seg, 0, ALINHA_TEMPO_MAX_S));
    }
    else if (n >= 1 && (!strcmp(acao, "off") || !strcmp(acao, "0"))) estados_alinha(false, 0);
    else if (n == 0 || n == -1) Log.printf("# alinha %s | fase %u, erro %.1f graus, %u s, %u pulsos (uso: alinha on [S] | alinha off)\n",
                                           estados_alinha_ativo() ? "LIGADO" : "desligado", (unsigned)estados_alinha_fase(),
                                           estados_alinha_erro(), (unsigned)estados_alinha_restante_s(), (unsigned)estados_alinha_pulsos());
    else Log.println(F("! uso: alinha on [S] | alinha off"));
  }
#if ANTI_TRAVA
  else if (!strcmp(c, "trava") || !strncmp(c, "trava ", 6)) {
    int m = -1;
    if (c[5] == ' ' && sscanf(c + 6, "%d", &m) == 1 && m >= 0 && m < MT_N) antitrava_define((uint8_t)m);
    else if (c[5] == ' ') Log.println(F("! uso: trava N (0 nenhuma, 1 lateral, 2 torque, 3 zigue, 4 giro, 5 pulso)"));
    Log.printf("# manobra do empurrao travado: %u %s\n", antitrava_manobra(), antitrava_nome(antitrava_manobra()));
  }
#endif
  else if (!strcmp(c, "ajuste") || !strncmp(c, "ajuste ", 7)) ajuste(c + (c[6] ? 7 : 6));
  else if (!strcmp(c, "go"))     { estados_iniciar(); Log.println(F("# start: contagem de 5 s")); }
  else if (!strcmp(c, "stop"))   { estados_parar();   Log.println(F("# parado")); }
  else if (!strcmp(c, "s1") || !strcmp(c, "geo") || !strcmp(c, "cfg")) { modoStream = modoStream || !strcmp(c, "s1"); streamCabecalho(); }
  else if (!strcmp(c, "s0"))     modoStream = false;
  else if (!strcmp(c, "wifi"))     telemetria_wifi_info();
  else if (!strcmp(c, "wifi off")) { if (telemetria_wifi_ligado()) telemetria_wifi_desliga("comando wifi off"); else telemetria_wifi_info(); }
  else Log.printf("! comando desconhecido: %s (digite help)\n", c);
}

void debug_init() { ajuda(); lista(); ajustesImprime(); }   // no boot aparecem os comandos, a tabela de sensores e as opções do painel

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
  if (tReinicia && (int32_t)(t - tReinicia) >= 0) { Serial.flush(); delay(50); ESP.restart(); }
  if (modoVer    && t - tVer    >= 200) { tVer = t;    imprimeVer(); }
  if (modoId     && t - tId     >= 50)  { tId = t;     imprimeId(); }
  if (modoBorda  && t - tBorda  >= 200) { tBorda = t;  imprimeBorda(); }
  if (modoStream && t - tStream >= STREAM_MS) { tStream = t; streamDados(); }
  if (modoStream && oponente_bayes() && t - tMapa >= MAPA_MS) { tMapa = t; streamMapa(); }
}
