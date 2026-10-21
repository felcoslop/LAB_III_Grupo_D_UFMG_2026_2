// teste_borda.cpp
// Testes do borda.cpp no computador: confirmação da linha branca, limiar da calibração de três medidas,
// limiar de pista gravado pelo painel (bordalim), persistência entre dois boots, sensor de polaridade
// invertida e as janelas de mínimo, máximo e média do comando calpista. As leituras do ADC e a
// memória não volátil (Preferences) são de mentira, controladas pelo teste.
// Uso (uma linha só):
//   g++ -std=c++11 -Wall -Wextra -I teste/unidade/stub -I firmware/sumo_esp32 teste/unidade/teste_borda.cpp firmware/sumo_esp32/borda.cpp -o t
//   ./t
#include <cstdio>
#include <cstdlib>
#include "Preferences.h"
#include "borda.h"
#include "config.h"

uint32_t relogio_us = 0;
static uint32_t mvPino[40];                    // o que o ADC de mentira devolve em cada GPIO
void analogReadResolution(int) {}
uint32_t analogReadMilliVolts(uint8_t pino) { return mvPino[pino]; }

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor = 0) {
  printf("%s  %-74s %8.1f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}
static void poe(uint8_t i, uint32_t mv) { mvPino[BORDA[i].pino] = mv; }
static void poeTodos(uint32_t mv) { for (uint8_t i = 0; i < N_BORDA; i++) poe(i, mv); }

static Borda b;
// uma leitura a cada 1 ms
static void passo(int ms) { for (int k = 0; k < ms; k++) { relogio_us += 1000; borda_atualiza(b); } }

int main() {
  static_assert(N_BORDA == 4, "o teste espera os 4 sensores de borda");
  poeTodos(3000);
  borda_init();

  // ---- sem calibração: limiar fixo do config.h
  confere(!borda_cal_gravada(0) && borda_limiar(0) == BORDA_LIMIAR_MV, "sem calibracao vale o limiar fixo do config.h", borda_limiar(0));
  confere(!borda_cal_limiar_gravado(0), "sem calibracao nao ha limiar de pista");

  // ---- confirmação: uma leitura isolada no branco (arranhão) não vale; branco contínuo vale
  passo(5);
  poe(0, 300); passo(1); poeTodos(3000); passo(5);
  confere(!b.detectada, "uma leitura de 1 ms no branco (arranhao) nao confirma a borda");
  poe(0, 300); passo(2);
  confere(!b.detectada && (b.cru & 1), "branco por 2 ms: cru ligado, ainda sem confirmar");
  passo(3);
  confere(b.detectada && (b.mascara & 1) && b.lado == -1 && b.frente, "branco por 5 ms confirma a borda (esquerda, frente)");
  poeTodos(3000); passo(2);
  confere(!b.detectada, "voltando ao preto a borda some na hora");

  // ---- calibração de três medidas: limiar = branco + fração x (preto - branco)
  borda_cal_define(0, 3000, 400);
  confere(borda_cal_gravada(0) && !borda_cal_limiar_gravado(0), "calborda grava preto e branco, sem limiar de pista");
  uint16_t esperado = (uint16_t)(400 + BORDA_FRACAO * 2600);
  confere(borda_limiar(0) == esperado, "limiar sai da fracao entre branco e preto", borda_limiar(0));

  // ---- calibração de pista: o limiar gravado vale no lugar da fração
  borda_cal_limiar(1, 1500, 3000, 400);
  confere(borda_cal_gravada(1) && borda_cal_limiar_gravado(1), "bordalim grava o limiar de pista");
  confere(borda_limiar(1) == 1500, "o limiar em uso e o gravado, nao o da fracao", borda_limiar(1));
  confere(borda_branco_abaixo(1), "preto 3000 e branco 400: branco = tensao baixa");
  poe(1, 1400); passo(6);
  confere(b.detectada && (b.mascara & 2), "leitura abaixo do limiar de pista (1400 < 1500) confirma o branco");
  poe(1, 1600); passo(3);
  confere(!b.detectada, "leitura acima do limiar de pista (1600) e preto");
  poeTodos(3000);

  // ---- persistência: um novo boot (borda_init) lê o que foi gravado
  borda_init();
  confere(borda_limiar(0) == esperado && !borda_cal_limiar_gravado(0), "depois do boot, sensor 0 mantem a calibracao de tres medidas", borda_limiar(0));
  confere(borda_limiar(1) == 1500 && borda_cal_limiar_gravado(1), "depois do boot, sensor 1 mantem o limiar de pista", borda_limiar(1));
  confere(!borda_cal_gravada(2) && borda_limiar(2) == BORDA_LIMIAR_MV, "sensor 2 segue sem calibracao");

  // ---- calborda por cima de uma calibração de pista tira o limiar de pista
  borda_cal_define(1, 3100, 500);
  confere(!borda_cal_limiar_gravado(1) && borda_limiar(1) == (uint16_t)(500 + BORDA_FRACAO * 2600), "calborda substitui a calibracao de pista", borda_limiar(1));
  borda_init();
  confere(!borda_cal_limiar_gravado(1), "e isso tambem vale depois do boot");

  // ---- sensor de polaridade invertida (branco = tensão alta)
  borda_cal_limiar(3, 2100, 400, 3000);
  confere(!borda_branco_abaixo(3) && borda_limiar(3) == 2100, "preto 400 e branco 3000: branco = tensao alta");
  poeTodos(3000); poe(3, 2500); passo(6);                // os outros sensores no preto (3000 mV)
  confere(b.detectada && b.mascara == 8, "modulo invertido confirma o branco acima do limiar (so ele)", b.mascara);
  poeTodos(3000); poe(3, 400); passo(3);
  confere(!b.detectada, "e o preto baixo (400 mV) nao confirma");

  // ---- bordazera apaga tudo
  borda_cal_zera();
  confere(!borda_cal_gravada(0) && !borda_cal_gravada(3) && !borda_cal_limiar_gravado(3) && borda_limiar(3) == BORDA_LIMIAR_MV,
          "bordazera volta ao limiar fixo em todos os sensores");
  borda_init();
  confere(!borda_cal_gravada(3), "e continua apagado depois do boot");

  // ---- janelas do calpista: mínimo, máximo e média de cada sensor desde o último fechamento
  poeTodos(3000); passo(3);
  uint16_t mn[N_BORDA], mx[N_BORDA], md[N_BORDA];
  confere(!borda_pista_ativa(), "calpista comeca desligado");
  confere(borda_pista_janela(mn, mx, md) == 0, "desligado nao junta nada");
  borda_pista(true);
  confere(borda_pista_ativa(), "calpista liga");
  const uint32_t seqA[] = {1000, 1200, 800, 1000};
  const uint32_t seqB[] = {3000, 2000, 3500, 3200};
  for (int k = 0; k < 4; k++) { poe(0, seqA[k]); poe(1, seqB[k]); poe(2, 500); poe(3, 4000); passo(1); }
  uint16_t n = borda_pista_janela(mn, mx, md);
  confere(n == 4, "a janela conta 4 leituras", n);
  confere(mn[0] == 800 && mx[0] == 1200 && md[0] == 1000, "sensor 0: minimo 800, maximo 1200, media 1000", md[0]);
  confere(mn[1] == 2000 && mx[1] == 3500 && md[1] == 2925, "sensor 1: minimo 2000, maximo 3500, media 2925", md[1]);
  confere(mn[2] == 500 && mx[2] == 500 && md[2] == 500 && mn[3] == 4000 && md[3] == 4000, "sensores constantes: minimo, maximo e media iguais");
  n = borda_pista_janela(mn, mx, md);
  confere(n == 0 && mn[0] == 0 && mx[0] == 0, "fechar a janela zera os acumuladores", n);
  poe(0, 111); passo(1);
  n = borda_pista_janela(mn, mx, md);
  confere(n == 1 && mn[0] == 111 && mx[0] == 111, "a janela seguinte comeca limpa (nao herda a anterior)", mn[0]);
  borda_pista(false);
  poe(0, 222); passo(3);
  confere(borda_pista_janela(mn, mx, md) == 0, "calpista off para de juntar");

  printf("\n%s: %d falha(s)\n", falhas ? "ERRO" : "OK", falhas);
  return falhas ? 1 : 0;
}
