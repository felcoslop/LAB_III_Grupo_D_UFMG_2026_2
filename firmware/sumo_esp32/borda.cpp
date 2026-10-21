// borda.cpp
// Leitura dos 4 TCRT5000 pela saída analógica A0 de cada módulo.
// A ideia segue o capítulo 9 do Blackbook: a leitura é analógica e passa por uma confirmação.
// Arranhão e poeira geram valores intermediários ou muito rápidos, enquanto a linha branca de
// verdade (2,5 cm) fica perto do valor do branco por alguns milissegundos. Por isso o limiar
// calibrado fica perto do branco (BORDA_FRACAO) e não no meio, e a borda só é aceita com
// branco contínuo por BORDA_CONFIRMA_MS e pelo menos 2 leituras seguidas.
// Existem duas calibrações gravadas: a de três medidas (calborda, guarda o preto e o branco e o
// limiar sai da fração) e a de pista (painel, comando bordalim), que guarda o limiar já pronto,
// calculado com o pior preto e o pior branco de uma volta inteira pelo dojô. Enquanto o comando
// calpista está ligado, o módulo ainda junta o mínimo, o máximo e a média de cada sensor em
// janelas, para o painel enxergar o branco e o preto de verdade e não só uma amostra a cada 20 ms.

#include "borda.h"
#include "config.h"
#include <Preferences.h>

static uint16_t mv[N_BORDA];                          // última leitura de cada sensor em mV
static uint16_t preto[N_BORDA], branco[N_BORDA], limiar[N_BORDA], limGrav[N_BORDA];
static bool     abaixo[N_BORDA], calOk[N_BORDA];      // sentido do branco e se existe calibração gravada
static bool     limOk[N_BORDA];                       // true quando o limiar gravado é o da calibração de pista
// acumuladores do comando calpista (uma janela por envio do painel)
static bool     pistaLigada = false;
static uint16_t jMin[N_BORDA], jMax[N_BORDA], jN = 0;
static uint32_t jSoma[N_BORDA];
static uint8_t  cont[N_BORDA] __attribute__((unused));   // leituras seguidas no branco
static uint32_t tIni[N_BORDA] __attribute__((unused));   // momento em que o branco começou
// o atributo unused só evita aviso do compilador quando USAR_BORDA vale 0

// Cálculo do limiar do sensor i. Com a calibração de pista, vale o limiar gravado. Com a de três
// medidas, o limiar fica entre o branco e o preto, a uma fração BORDA_FRACAO a partir do branco.
// Sem calibração, valem os valores fixos do config.h.
static void recalcula(uint8_t i) {
  if (calOk[i]) {
    abaixo[i] = branco[i] < preto[i];
    limiar[i] = limOk[i] ? limGrav[i] : (uint16_t)(branco[i] + BORDA_FRACAO * ((int)preto[i] - (int)branco[i]));
  } else {
    abaixo[i] = BORDA_BRANCO_ABAIXO;
    limiar[i] = BORDA_LIMIAR_MV;
  }
}

// Configuração dos pinos e leitura da calibração salva no namespace "borda" das Preferences.
// Cada sensor usa até três chaves, por exemplo "BFE_p" (preto), "BFE_b" (branco) e, só na
// calibração de pista, "BFE_l" (limiar).
void borda_init() {
#if USAR_BORDA
  analogReadResolution(12);
  Preferences pr;
  pr.begin("borda", true);          // true abre só para leitura
  for (uint8_t i = 0; i < N_BORDA; i++) {
    pinMode(BORDA[i].pino, INPUT);
    char kp[12], kb[12], kl[12];
    snprintf(kp, sizeof(kp), "%s_p", BORDA[i].nome);
    snprintf(kb, sizeof(kb), "%s_b", BORDA[i].nome);
    snprintf(kl, sizeof(kl), "%s_l", BORDA[i].nome);
    calOk[i] = pr.isKey(kp) && pr.isKey(kb);    // o sensor só conta como calibrado quando as duas chaves existem
    preto[i]  = calOk[i] ? pr.getUShort(kp, 0) : 0;
    branco[i] = calOk[i] ? pr.getUShort(kb, 0) : 0;
    limOk[i]  = calOk[i] && pr.isKey(kl);
    limGrav[i] = limOk[i] ? pr.getUShort(kl, 0) : 0;
    recalcula(i);
    mv[i] = 0; cont[i] = 0;
  }
  pr.end();
#endif
}

// Gravação da calibração do sensor i, pedida pelo comando calborda do debug.cpp; o limiar novo já passa a valer.
// Uma calibração de três medidas substitui a de pista (o limiar gravado sai).
void borda_cal_define(uint8_t i, uint16_t p, uint16_t b) {
  if (i >= N_BORDA) return;
  Preferences pr;
  pr.begin("borda", false);
  char kp[12], kb[12], kl[12];
  snprintf(kp, sizeof(kp), "%s_p", BORDA[i].nome);
  snprintf(kb, sizeof(kb), "%s_b", BORDA[i].nome);
  snprintf(kl, sizeof(kl), "%s_l", BORDA[i].nome);
  pr.putUShort(kp, p);
  pr.putUShort(kb, b);
  pr.remove(kl);
  pr.end();
  preto[i] = p; branco[i] = b; calOk[i] = true; limOk[i] = false;
  recalcula(i);
}

// Gravação da calibração de pista do sensor i (comando bordalim): o limiar já vem calculado pelo painel,
// com o pior preto e o pior branco de uma volta inteira, e vale no lugar da fração BORDA_FRACAO. O preto e o
// branco gravados são só os valores típicos, para o sentido do sensor (qual dos dois lê tensão menor) e para a tabela.
void borda_cal_limiar(uint8_t i, uint16_t lim, uint16_t p, uint16_t b) {
  if (i >= N_BORDA) return;
  Preferences pr;
  pr.begin("borda", false);
  char kp[12], kb[12], kl[12];
  snprintf(kp, sizeof(kp), "%s_p", BORDA[i].nome);
  snprintf(kb, sizeof(kb), "%s_b", BORDA[i].nome);
  snprintf(kl, sizeof(kl), "%s_l", BORDA[i].nome);
  pr.putUShort(kp, p);
  pr.putUShort(kb, b);
  pr.putUShort(kl, lim);
  pr.end();
  preto[i] = p; branco[i] = b; limGrav[i] = lim; calOk[i] = true; limOk[i] = true;
  recalcula(i);
}

// O comando bordazera usa esta função para apagar toda a calibração e voltar ao limiar fixo.
void borda_cal_zera() {
  Preferences pr;
  pr.begin("borda", false);
  pr.clear();
  pr.end();
  for (uint8_t i = 0; i < N_BORDA; i++) { calOk[i] = false; limOk[i] = false; recalcula(i); }
}

// Funções de consulta usadas pelo debug.cpp; todas conferem o índice antes de acessar o vetor.
bool     borda_cal_gravada(uint8_t i)   { return i < N_BORDA && calOk[i]; }
bool     borda_cal_limiar_gravado(uint8_t i) { return i < N_BORDA && limOk[i]; }
uint16_t borda_cal_preto(uint8_t i)     { return i < N_BORDA ? preto[i] : 0; }
uint16_t borda_cal_branco(uint8_t i)    { return i < N_BORDA ? branco[i] : 0; }
uint16_t borda_limiar(uint8_t i)        { return i < N_BORDA ? limiar[i] : 0; }
bool     borda_branco_abaixo(uint8_t i) { return i < N_BORDA && abaixo[i]; }
uint16_t borda_mv(uint8_t i)            { return i < N_BORDA ? mv[i] : 0; }

// Janelas do comando calpista. Ligar zera os acumuladores; a cada envio do painel a janela é fechada e
// devolve quantas voltas do loop entraram e o mínimo, o máximo e a média (mV) de cada sensor.
void borda_pista(bool liga) {
  pistaLigada = liga;
  jN = 0;
  for (uint8_t i = 0; i < N_BORDA; i++) { jMin[i] = 0xFFFF; jMax[i] = 0; jSoma[i] = 0; }
}
bool borda_pista_ativa() { return pistaLigada; }
uint16_t borda_pista_janela(uint16_t* mn, uint16_t* mx, uint16_t* med) {
  uint16_t n = jN;
  for (uint8_t i = 0; i < N_BORDA; i++) {
    mn[i] = n ? jMin[i] : 0; mx[i] = n ? jMax[i] : 0; med[i] = n ? (uint16_t)((jSoma[i] + n / 2) / n) : 0;
    jMin[i] = 0xFFFF; jMax[i] = 0; jSoma[i] = 0;
  }
  jN = 0;
  return n;
}

// Comparação da última leitura com o limiar, respeitando o sentido do sensor.
bool borda_branco_cru(uint8_t i) {
  if (i >= N_BORDA) return false;
  return abaixo[i] ? mv[i] < limiar[i] : mv[i] > limiar[i];
}

// O percepcao_atualiza() chama esta função em toda volta do loop. O laço lê cada sensor, conta há
// quanto tempo ele está no branco e marca na máscara só os que passaram pela confirmação.
// No final a máscara vira as informações que a máquina de estados usa (lado, frente, trás).
void borda_atualiza(Borda& b) {
  b.detectada = false; b.mascara = 0; b.cru = 0; b.lado = 0; b.frente = true; b.tras = false;
#if USAR_BORDA
  uint32_t agora = millis();
  bool esq = false, dir = false, fr = false, tr = false;
  for (uint8_t i = 0; i < N_BORDA; i++) {
    mv[i] = analogReadMilliVolts(BORDA[i].pino);   // o ESP32 já devolve em mV usando a calibração de fábrica do ADC
    if (pistaLigada) {                             // acumuladores da janela do comando calpista
      if (mv[i] < jMin[i]) jMin[i] = mv[i];
      if (mv[i] > jMax[i]) jMax[i] = mv[i];
      jSoma[i] += mv[i];
    }
    if (borda_branco_cru(i)) {
      b.cru |= (1u << i);
      if (cont[i] == 0) tIni[i] = agora;           // primeira leitura no branco guarda o início
      if (cont[i] < 255) cont[i]++;                // o contador para em 255 para não estourar o uint8_t
    } else cont[i] = 0;                            // qualquer leitura no preto zera a contagem
    if (cont[i] >= 2 && agora - tIni[i] >= BORDA_CONFIRMA_MS) {
      b.mascara |= (1u << i);
      if (BORDA[i].lado < 0) esq = true; else dir = true;
      if (BORDA[i].frente) fr = true; else tr = true;
    }
  }
  if (pistaLigada && jN < 0xFFFF) jN++;
  b.detectada = b.mascara != 0;
  b.lado = (esq && dir) ? 0 : esq ? -1 : dir ? 1 : 0;   // 0 quando os dois lados (ou nenhum) veem branco
  b.frente = fr;
  b.tras = tr;
#endif
}
