// teste_ajustes.cpp
// Testes das opções do painel gravadas na NVS (ajustes.cpp), no computador, com a NVS de mentira dos stubs: valor de
// fábrica, validação, gravação que sobrevive a um "boot" (ajustes_carrega), a quantidade de ToF que só muda depois de
// reiniciar, o volta-ao-padrão e a posição dos sensores de linha da frente (borda_x e borda_y) nas duas disposições.
// Uso (uma linha só, FW = firmware/sumo_esp32):
//   g++ -std=c++11 -Wall -Wextra -I teste/unidade/stub -I $FW teste/unidade/teste_ajustes.cpp $FW/ajustes.cpp -o t && ./t
#include <cstdio>
#include "ajustes.h"

uint32_t relogio_us = 0;

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor) {
  printf("%s  %-76s %9.2f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}
static void boot() { ajustes_carrega(); }   // o setup do robô lê a NVS antes de qualquer módulo

int main() {
  // a tabela sempre traz os sete sensores; os dois últimos são o par de trás
  confere(TOF_MAX == 7, "a tabela de ToF tem sempre 7 sensores", TOF_MAX);
  confere(!strcmp(TOF[5].nome, "TE") && !strcmp(TOF[6].nome, "TD"), "e os dois últimos são TE e TD", 0);
  confere(GEOMETRIA_BANCADA == 0, "GEOMETRIA_BANCADA vale 0", GEOMETRIA_BANCADA);

  // valor de fábrica
  boot();
  confere(ajustes().tof == QTD_TOF && (uint8_t)N_TOF == QTD_TOF, "fábrica: quantidade de ToF igual ao QTD_TOF do config.h", N_TOF);
  confere(ajustes().autonomo == (AUTO_START != 0), "fábrica: modo autônomo igual ao AUTO_START do config.h", ajustes().autonomo);
  confere(!ajustes().borda_lateral && !ajustes().alinha_contagem && !ajustes().wifi_luta, "fábrica: linha no CAD, contagem parada e Wi-Fi desligado na luta", 0);
  confere(ajustes().wifi_leve == (WIFI_LEVE != 0), "fábrica: Wi-Fi leve igual ao WIFI_LEVE do config.h", ajustes().wifi_leve);
  confere(!ajustes_gravado(), "fábrica: nada gravado na NVS", 0);

  // validação
  confere(ajustes_define("tof", 6) == AJUSTE_INVALIDO, "tof 6 é inválido", 6);
  confere(ajustes_define("auto", 2) == AJUSTE_INVALIDO, "auto 2 é inválido", 2);
  confere(ajustes_define("linha", -1) == AJUSTE_INVALIDO, "linha -1 é inválido", -1);
  confere(ajustes_define("nada", 1) == AJUSTE_INVALIDO, "nome desconhecido é inválido", 0);
  confere(!ajustes_gravado(), "e nenhuma recusa grava alguma coisa", 0);

  // opções que valem na hora
  confere(ajustes_define("auto", 1) == AJUSTE_OK && ajustes().autonomo, "auto 1: grava e vale", ajustes().autonomo);
  confere(ajustes_define("linha", 1) == AJUSTE_OK && ajustes().borda_lateral, "linha 1: grava e vale", ajustes().borda_lateral);
  confere(ajustes_define("ALINHA", 1) == AJUSTE_OK && ajustes().alinha_contagem, "alinha 1 (maiúsculas): grava e vale", ajustes().alinha_contagem);
  confere(ajustes_define("wifi", 1) == AJUSTE_OK && ajustes().wifi_luta, "wifi 1: grava e vale", ajustes().wifi_luta);
  const bool leveFabrica = WIFI_LEVE != 0;
  confere(ajustes_define("leve", leveFabrica ? 0 : 1) == AJUSTE_OK && ajustes().wifi_leve == !leveFabrica, "leve (contrário da fábrica): grava e vale", ajustes().wifi_leve);
  confere(ajustes_gravado(), "agora existe ajuste gravado", 0);

  // sobrevive ao boot
  ajustes() = Ajustes{QTD_TOF, false, false, false, false, leveFabrica};     // RAM zerada pelo reset
  boot();
  confere(ajustes().autonomo && ajustes().borda_lateral && ajustes().alinha_contagem && ajustes().wifi_luta && ajustes().wifi_leve == !leveFabrica,
          "depois do boot as cinco opções voltam da NVS", 0);
  confere(ajustes_define("auto", 0) == AJUSTE_OK && !ajustes().autonomo, "auto 0 desliga", ajustes().autonomo);
  boot();
  confere(!ajustes().autonomo && ajustes().borda_lateral, "e o desligado também fica gravado", ajustes().autonomo);

  // quantidade de ToF: só vale depois de reiniciar
  const uint8_t outra = QTD_TOF == 5 ? 7 : 5;
  confere(ajustes_define("tof", QTD_TOF) == AJUSTE_OK, "tof igual ao em uso: não pede reiniciar", QTD_TOF);
  confere(ajustes_define("tof", outra) == AJUSTE_REINICIA, "tof diferente do em uso: pede reiniciar", outra);
  confere((uint8_t)N_TOF == QTD_TOF, "e a quantidade em uso só muda no boot", N_TOF);
  confere(ajustes_define("tof", QTD_TOF) == AJUSTE_OK, "voltar ao valor em uso cancela o pedido", QTD_TOF);
  boot();
  confere((uint8_t)N_TOF == QTD_TOF, "depois do boot continua a quantidade original", N_TOF);
  ajustes_define("tof", outra);
  boot();
  confere((uint8_t)N_TOF == outra, "depois do boot a quantidade gravada passa a valer", N_TOF);
  confere(N_TOF <= TOF_MAX, "e cabe na tabela", N_TOF);

  // sensores de linha da frente
  ajustes().borda_lateral = false;
  confere(borda_x(0) == BORDA[0].x && borda_y(0) == BORDA[0].y, "linha no CAD: borda_x e borda_y são os da tabela (BFE)", borda_y(0));
  ajustes().borda_lateral = true;
  bool achouE = false, achouD = false, trasIgual = true;
  for (uint8_t i = 0; i < N_BORDA; i++) {
    if (BORDA[i].frente && BORDA[i].lado < 0) { achouE = borda_x(i) == BORDA_LATERAL_X_MM && borda_y(i) == BORDA_LATERAL_Y_MM; }
    if (BORDA[i].frente && BORDA[i].lado > 0) { achouD = borda_x(i) == BORDA_LATERAL_X_MM && borda_y(i) == -BORDA_LATERAL_Y_MM; }
    if (!BORDA[i].frente && (borda_x(i) != BORDA[i].x || borda_y(i) != BORDA[i].y)) trasIgual = false;
  }
  confere(achouE, "colados de lado: o da frente esquerda vai para y positivo (esquerda)", BORDA_LATERAL_Y_MM);
  confere(achouD, "colados de lado: o da frente direita vai para y negativo (direita)", -BORDA_LATERAL_Y_MM);
  confere(trasIgual, "colados de lado: os de trás não mudam", 0);

  // padrão de fábrica
  ajustes_define("auto", 1); ajustes_define("wifi", 1);
  const int8_t r = ajustes_padrao();
  confere(!ajustes_gravado() && ajustes().autonomo == (AUTO_START != 0) && !ajustes().borda_lateral && !ajustes().wifi_luta && ajustes().wifi_leve == leveFabrica,
          "ajuste padrao: apaga tudo e volta ao valor de fábrica", 0);
  confere(r == AJUSTE_REINICIA && (uint8_t)N_TOF == outra, "ajuste padrao com outra quantidade de ToF em uso: pede reiniciar e mantém a em uso", N_TOF);
  boot();
  confere((uint8_t)N_TOF == QTD_TOF, "e depois do boot volta à quantidade de fábrica", N_TOF);

  printf(falhas ? "\nERRO: %d falha(s)\n" : "\nOK: 0 falha(s)\n", falhas);
  return falhas ? 1 : 0;
}
