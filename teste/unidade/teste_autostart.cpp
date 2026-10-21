// teste_autostart.cpp
// Testes da partida automática (AUTO_START 1), no computador. O robô sem botão de início precisa sair
// da ESPERA sozinho, mas só com os sensores prontos, só depois de AUTO_START_ESPERA_MS, uma única vez
// por ligada, e nunca depois do comando stop. A percepção P é montada à mão, como em teste_estados.cpp.
// Uso (uma linha só):
//   g++ -std=c++11 -Wall -Wextra -DAUTO_START=1 -I teste/unidade/stub -I . teste/unidade/teste_autostart.cpp estados.cpp cinematica.cpp arena.cpp chave.cpp antitrava.cpp -o t
//   ./t
#include <cstdio>
#include <cstdlib>
#include "estados.h"
#include "percepcao.h"
#include "cinematica.h"
#include "telemetria.h"
#include "arena.h"
#include "chave.h"

#if !AUTO_START
#error "este teste precisa de -DAUTO_START=1"
#endif

uint32_t relogio_us = 0;
Percepcao P;
static int16_t pwmE = 0, pwmD = 0;
void motores(int16_t esq, int16_t dir) { pwmE = esq; pwmD = dir; }
void motores_para() { motores(0, 0); }
size_t Saida::write(uint8_t) { return 1; }
size_t Saida::write(const uint8_t*, size_t n) { return n; }
Saida Log;

// chave DIP de mentira, sem nenhuma via ligada (o robô sem a chave), e perfil dos sensores aplicado na hora
int digitalRead(int) { return 0; }
static bool perfilEscuro = false;
void tof_perfil(bool escuro) { perfilEscuro = escuro; }
bool tof_perfil_escuro() { return perfilEscuro; }
bool oponente_borda_tras_recente() { return false; }

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor) {
  printf("%s  %-66s %9.1f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}
// uma volta do loop a cada 2 ms, com a foto atual em P
static void passo(int ms) {
  for (int t = 0; t < ms; t += 2) { relogio_us += 2000; P.t_ms = millis(); chave_atualiza(); estados_passo(); P.start = false; }
}

int main() {
  memset(&P, 0, sizeof(P));
  cinematica_init(); estados_init(); chave_init(); arena_init();
  P.arena = Arena{};

  // sem sensores prontos o robô fica parado, mesmo depois de muito tempo ligado
  P.saude.pronto = false;
  passo(4000);
  confere(estado_atual() == ESPERA, "sensores nao prontos: fica na ESPERA depois de 4 s", estado_atual());

  // sensores prontos: espera AUTO_START_ESPERA_MS antes de pedir o start
  P.saude.pronto = true;
  passo(AUTO_START_ESPERA_MS - 100);
  confere(estado_atual() == ESPERA, "sensores prontos ha menos de AUTO_START_ESPERA_MS: ainda ESPERA", estado_atual());
  passo(200);
  confere(estado_atual() == CONTAGEM, "AUTO_START_ESPERA_MS cumprido: entra na CONTAGEM sozinho", estado_atual());
  confere(pwmE == 0 && pwmD == 0, "CONTAGEM: motores parados", pwmE);

  // a contagem de 5 s da regra vale do mesmo jeito
  passo(T_CONTAGEM_MS - 400);
  confere(estado_atual() == CONTAGEM, "CONTAGEM ainda segue antes dos 5 s", estado_atual());
  passo(500);
  confere(estado_atual() == BUSCA, "depois de 5 s: BUSCA", estado_atual());

  // o stop volta para a ESPERA e o robô fica lá: a partida automática vale uma vez por ligada
  estados_parar();
  passo(4000);
  confere(estado_atual() == ESPERA, "stop: volta para a ESPERA e nao parte de novo sozinho", estado_atual());

  // o comando go (ou o botão) continua valendo depois do stop
  estados_iniciar();
  passo(10);
  confere(estado_atual() == CONTAGEM, "go depois do stop: CONTAGEM", estado_atual());

  printf("\n%s: %d falha(s)\n", falhas ? "ERRO" : "OK", falhas);
  return falhas ? 1 : 0;
}
