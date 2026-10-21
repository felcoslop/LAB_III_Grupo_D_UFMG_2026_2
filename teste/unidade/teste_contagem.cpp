// teste_contagem.cpp
// Testes da contagem de 5 s com os ajustes do painel, da máquina de estados de verdade no computador (o mesmo mundo mínimo
// do teste_alinha.cpp: o robô gira conforme a cinemática estima e os ToF da frente só veem a até 32 graus do centro).
// Confere que, sem o ajuste "alinha", o robô fica parado nos 5 s (regra da CoRA); com ele, só gira (nunca anda), chega
// de frente para o oponente e a luta começa na hora; que o BOOT recomeça a contagem (e, com o modo autônomo ligado,
// interrompe a luta e recomeça); e que o modo autônomo ligado no painel só larga depois de o robô ser ligado de novo.
// Uso (uma linha só, FW = firmware/sumo_esp32):
//   g++ -std=c++11 -Wall -Wextra -I teste/unidade/stub -I $FW teste/unidade/teste_contagem.cpp $FW/estados.cpp
//       $FW/cinematica.cpp $FW/arena.cpp $FW/chave.cpp $FW/antitrava.cpp -o t && ./t
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "estados.h"
#include "percepcao.h"
#include "cinematica.h"
#include "telemetria.h"
#include "arena.h"
#include "chave.h"

uint32_t relogio_us = 0;
Percepcao P;
static int16_t pwmE = 0, pwmD = 0;
void motores(int16_t esq, int16_t dir) { pwmE = esq; pwmD = dir; }
void motores_para() { motores(0, 0); }
static std::string texto;
size_t Saida::write(uint8_t c) { texto.push_back((char)c); return 1; }
size_t Saida::write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
Saida Log;
int digitalRead(int) { return 0; }
void tof_perfil(bool) {}
bool tof_perfil_escuro() { return false; }
bool oponente_borda_tras_recente() { return false; }

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor) {
  printf("%s  %-76s %9.2f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}

static float psi = 0, beta = 0;
static bool  oponentePresente = true;
static float giroTotal = 0, vMax = 0, somaPwmMax = 0;
static float ultE = 0; static int8_t ultLado = 0; static uint32_t tUltVisto = 0;
static float eMedido = 0; static bool veMedido = false, veAnterior = false; static float eAnterior = 0;
static float envolve(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
static void passo(int ms = 2) {
  for (int t = 0; t < ms; t += 2) {
    relogio_us += 2000; P.t_ms = millis();
    if ((P.t_ms % 30) < 2) {
      veAnterior = veMedido; eAnterior = eMedido;
      float e = envolve(beta - psi);
      veMedido = oponentePresente && fabsf(e) <= 32.0f; eMedido = e;
      P.op.visto = veAnterior; P.op.centrado = veAnterior && fabsf(eAnterior) <= CENTRO_DEG;
      if (veAnterior) { P.op.angulo = eAnterior; P.op.dist_mm = 300; P.op.existe = 1; ultLado = eAnterior > 0 ? 1 : -1; tUltVisto = P.t_ms; ultE = eAnterior; }
      else { P.op.angulo = ultE; P.op.dist_mm = -1; P.op.existe = oponentePresente ? 0.4f : 0.0f; }
      P.op.lado_ultimo = ultLado;
    }
    P.op.ms_sem_ver = P.op.visto ? 0 : (tUltVisto ? P.t_ms - tUltVisto : 99999);
    estados_passo(); P.start = false;
    float w = cinematica_w_dps();
    psi += w * 0.002f; giroTotal += fabsf(w) * 0.002f;
    if (fabsf(cinematica_v_mms()) > vMax) vMax = fabsf(cinematica_v_mms());
    if (abs(pwmE + pwmD) > somaPwmMax) somaPwmMax = abs(pwmE + pwmD);
  }
}
static void zeraMedidas() { giroTotal = 0; vMax = 0; somaPwmMax = 0; }
static void reinicia(float rumoRobo, float rumoOponente) {
  estados_parar(); texto.clear();
  psi = rumoRobo; beta = rumoOponente; oponentePresente = true;
  zeraMedidas(); ultE = 0; ultLado = 0; tUltVisto = 0; veMedido = veAnterior = false;
  P.op = Oponente{}; P.borda = Borda{}; P.saude.pronto = true; P.start = false;
  passo(700);                                          // a velocidade estimada da luta anterior (modelo das rodas) termina de cair a zero
  zeraMedidas();
}
static bool contem(const char* s) { return texto.find(s) != std::string::npos; }
static void apertaBoot() { P.start = true; passo(2); }

int main() {
  memset(&P, 0, sizeof(P));
  cinematica_init(); chave_init(); arena_init();
  P.saude.pronto = true;

  // 0. modo autônomo ligado pelo painel com o robô já ligado: só larga no próximo boot
  ajustes().autonomo = false; estados_init();
  passo(50);
  passo(AUTO_START_ESPERA_MS + 1500);
  confere(estado_atual() == ESPERA, "modo autônomo desligado: o robô fica em ESPERA", estado_atual());
  ajustes().autonomo = true;                          // "ajuste auto 1" no meio da sessão
  passo(AUTO_START_ESPERA_MS + 1500);
  confere(estado_atual() == ESPERA, "ligou o modo autônomo agora: não larga na hora do clique", estado_atual());
  estados_init();                                     // o robô é ligado de novo
  passo(AUTO_START_ESPERA_MS / 2);
  confere(estado_atual() == ESPERA, "depois do boot espera os sensores ficarem prontos", estado_atual());
  passo(AUTO_START_ESPERA_MS + 200);
  confere(estado_atual() == CONTAGEM && contem("AUTO_START"), "depois do boot o modo autônomo faz a contagem sozinho", estado_atual());
  ajustes().autonomo = false;
  estados_parar();
  estados_init();
  passo(AUTO_START_ESPERA_MS + 1500);
  confere(estado_atual() == ESPERA, "modo autônomo desligado de novo: nada de partida automática", estado_atual());

  // 1. contagem sem o ajuste alinha: parado nos 5 s, mesmo com o oponente no lado
  ajustes().alinha_contagem = false;
  reinicia(0, 40);
  estados_iniciar(); passo(10);
  confere(estado_atual() == CONTAGEM, "go: começa a contagem", estado_atual());
  passo(T_CONTAGEM_MS - 300);
  confere(estado_atual() == CONTAGEM && giroTotal < 1.0f && vMax < 1.0f && pwmE == 0 && pwmD == 0, "sem o ajuste alinha: parado até o fim da contagem (graus girados)", giroTotal);
  passo(500);
  confere(estado_atual() >= BUSCA, "e a luta começa quando os 5 s acabam", estado_atual());

  // 2. com o ajuste alinha: só gira, chega de frente para o oponente e a luta começa na hora
  ajustes().alinha_contagem = true;
  reinicia(0, 40);
  estados_iniciar(); passo(10);
  passo(T_CONTAGEM_MS - 300);
  confere(estado_atual() == CONTAGEM && contem("so gira ate ficar de frente"), "com o ajuste alinha: avisa que só gira na contagem", estado_atual());
  confere(giroTotal > 20.0f, "girou em direção ao oponente (graus girados)", giroTotal);
  confere(vMax < 5.0f, "nunca andou: avanço estimado abaixo de 5 mm/s", vMax);
  confere(somaPwmMax <= 8, "as duas rodas sempre com PWM opostos (giro no eixo)", somaPwmMax);
  confere(fabsf(envolve(beta - psi)) <= ALINHA_TOL_DEG + 0.6f, "chegou de frente para o oponente antes de acabar a contagem (graus)", envolve(beta - psi));
  passo(500);
  confere(estado_atual() >= BUSCA, "a luta começa quando os 5 s acabam", estado_atual());
  confere(!estados_alinha_ativo() && estados_alinha_fase() == 0, "e o alinhamento da contagem é desligado", estados_alinha_fase());
  confere(estado_atual() == ATAQUE || estado_atual() == MIRA, "já largando de frente para o oponente: ataca (estado)", estado_atual());

  // 3. oponente atrás: procura em pulsos durante a contagem, sem andar
  reinicia(0, 180);
  estados_iniciar(); passo(10);
  passo(T_CONTAGEM_MS - 300);
  confere(giroTotal > 60.0f && vMax < 5.0f, "oponente atrás: procura girando e sem andar (graus girados)", giroTotal);
  estados_parar();

  // 4. oponente ausente: procura em pulsos e a luta começa mesmo assim ao fim dos 5 s
  reinicia(0, 0); oponentePresente = false;
  estados_iniciar(); passo(10);
  passo(T_CONTAGEM_MS + 300);
  confere(estado_atual() == BUSCA && vMax < 5.0f, "sem oponente: começa a luta em BUSCA depois dos 5 s", estado_atual());

  // 5. BOOT na contagem recomeça os 5 s
  ajustes().alinha_contagem = true;
  reinicia(0, 40);
  estados_iniciar(); passo(10);
  passo(3000);
  confere(estado_atual() == CONTAGEM, "3 s depois do go ainda em contagem", estado_atual());
  apertaBoot();
  confere(contem("contagem de 5 s reiniciada"), "BOOT na contagem: avisa que reiniciou", 0);
  passo(T_CONTAGEM_MS - 600);
  confere(estado_atual() == CONTAGEM, "4,4 s depois do BOOT ainda em contagem (sem o BOOT já teria largado)", estado_atual());
  passo(900);
  confere(estado_atual() >= BUSCA, "e larga 5 s depois do BOOT", estado_atual());

  // 6. BOOT sem alinhamento também recomeça, e o robô continua parado
  ajustes().alinha_contagem = false;
  reinicia(0, 40);
  estados_iniciar(); passo(10);
  passo(4000);
  apertaBoot();
  passo(2000);
  confere(estado_atual() == CONTAGEM && giroTotal < 1.0f, "sem alinha: BOOT recomeça a contagem e o robô segue parado", giroTotal);
  estados_parar();

  // 7. BOOT no meio da luta: só com o modo autônomo ligado
  ajustes().alinha_contagem = true;
  ajustes().autonomo = false;
  reinicia(0, 0);
  estados_iniciar(); passo(T_CONTAGEM_MS + 500);
  confere(estado_atual() >= BUSCA, "luta em andamento (modo autônomo desligado)", estado_atual());
  apertaBoot(); passo(20);
  confere(estado_atual() >= BUSCA, "BOOT na luta com o modo autônomo desligado: nada muda", estado_atual());
  estados_parar();
  ajustes().autonomo = true;
  reinicia(0, 0);
  estados_iniciar(); passo(T_CONTAGEM_MS + 500);
  confere(estado_atual() >= BUSCA, "luta em andamento (modo autônomo ligado)", estado_atual());
  apertaBoot(); passo(20);
  confere(estado_atual() == CONTAGEM && contem("luta interrompida"), "BOOT na luta com o modo autônomo ligado: volta à contagem", estado_atual());
  confere((pwmE == 0 && pwmD == 0) || estados_alinha_ativo(), "e o robô para de lutar", pwmE);
  beta = 30; const float psiBoot = psi;               // o oponente foi reposto na mão, 30 graus para o lado
  passo(700); zeraMedidas();                           // o robô ainda freava da luta: a medida começa depois
  passo(T_CONTAGEM_MS - 1000);
  confere(estado_atual() == CONTAGEM && vMax < 5.0f && fabsf(envolve(psi - psiBoot)) > 20.0f, "a nova contagem gira para o oponente sem andar (graus girados)", envolve(psi - psiBoot));
  confere(fabsf(envolve(beta - psi)) <= ALINHA_TOL_DEG + 0.6f, "e termina de frente para o oponente reposto (graus)", envolve(beta - psi));
  passo(1300);
  confere(estado_atual() >= BUSCA, "e a luta recomeça", estado_atual());
  ajustes().autonomo = false; ajustes().alinha_contagem = false;
  estados_parar();

  printf(falhas ? "\nERRO: %d falha(s)\n" : "\nOK: 0 falha(s)\n", falhas);
  return falhas ? 1 : 0;
}
