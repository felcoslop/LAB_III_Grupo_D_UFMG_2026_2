// teste_alinha.cpp
// Testes da demonstração de alinhamento (comando "alinha"), da máquina de estados de verdade no computador, com um
// mundo mínimo em volta: o robô gira no eixo conforme o giro que a cinemática estima, o oponente fica em um rumo
// fixo no mundo, e os ToF da frente só veem o que estiver a até 32 graus do centro (a cada 30 ms, com uma leitura
// de atraso). Confere que o robô só gira (nunca anda), fica alinhado a menos de ALINHA_TOL_DEG, acha o oponente
// que está de lado e atrás, não passa do ponto nem com as rodas girando mais do que o modelo, ignora a linha, acaba
// no tempo e sai ao receber go ou stop. Também confere as flags de linha ignorada e de texto calado dos testes mov e pwm.
// Uso (uma linha só, FW = firmware/sumo_esp32):
//   g++ -std=c++11 -Wall -Wextra -I teste/unidade/stub -I $FW teste/unidade/teste_alinha.cpp $FW/estados.cpp
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
static std::string texto;                       // tudo o que o firmware escreveu no terminal
size_t Saida::write(uint8_t c) { texto.push_back((char)c); return 1; }
size_t Saida::write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
Saida Log;
int digitalRead(int) { return 0; }
void tof_perfil(bool) {}
bool tof_perfil_escuro() { return false; }
bool oponente_borda_tras_recente() { return false; }

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor) {
  printf("%s  %-72s %9.2f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}

// ---- mundo ----
static float psi = 0;                           // rumo do robô no mundo, graus (positivo é esquerda)
static float beta = 0;                          // rumo do oponente no mundo
static float ganhoRoda = 1.0f;                  // 1 = as rodas giram o que a cinemática estima; 1,8 = giram demais
static bool  oponentePresente = true;
static float giroTotal = 0, vMax = 0, somaPwmMax = 0;
static float ultE = 0; static int8_t ultLado = 0; static uint32_t tUltVisto = 0;
static float eMedido = 0; static bool veMedido = false, veAnterior = false; static float eAnterior = 0;
static float envolve(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
// 2 ms por volta do loop; a cada 30 ms os ToF entregam uma leitura nova (a anterior ainda vale enquanto isso)
static void passo(int ms = 2) {
  for (int t = 0; t < ms; t += 2) {
    relogio_us += 2000; P.t_ms = millis();
    if ((P.t_ms % 30) < 2) {
      veAnterior = veMedido; eAnterior = eMedido;                       // atraso de uma leitura
      float e = envolve(beta - psi);
      veMedido = oponentePresente && fabsf(e) <= 32.0f; eMedido = e;
      P.op.visto = veAnterior; P.op.centrado = veAnterior && fabsf(eAnterior) <= CENTRO_DEG;
      if (veAnterior) { P.op.angulo = eAnterior; P.op.dist_mm = 300; P.op.existe = 1; ultLado = eAnterior > 0 ? 1 : -1; tUltVisto = P.t_ms; ultE = eAnterior; }
      else { P.op.angulo = ultE; P.op.dist_mm = -1; P.op.existe = oponentePresente ? 0.4f : 0.0f; }
      P.op.lado_ultimo = ultLado;
    }
    P.op.ms_sem_ver = P.op.visto ? 0 : (tUltVisto ? P.t_ms - tUltVisto : 99999);
    estados_passo(); P.start = false;
    float w = cinematica_w_dps() * ganhoRoda;
    psi += w * 0.002f; giroTotal += fabsf(w) * 0.002f;
    if (fabsf(cinematica_v_mms()) > vMax) vMax = fabsf(cinematica_v_mms());
    if (abs(pwmE + pwmD) > somaPwmMax) somaPwmMax = abs(pwmE + pwmD);
  }
}
static void reinicia(float rumoRobo, float rumoOponente, float ganho = 1.0f) {
  estados_parar(); texto.clear();
  psi = rumoRobo; beta = rumoOponente; ganhoRoda = ganho; oponentePresente = true;
  giroTotal = 0; vMax = 0; somaPwmMax = 0; ultE = 0; ultLado = 0; tUltVisto = 0; veMedido = veAnterior = false;
  P.op = Oponente{}; P.borda = Borda{}; P.saude.pronto = true; P.start = false;
  passo(10);
}
static bool contem(const char* s) { return texto.find(s) != std::string::npos; }
// roda até alinhar ou até ms; devolve o tempo gasto (ms) ou -1
static int ateAlinhar(int ms) {
  for (int t = 0; t < ms; t += 2) {
    passo();
    if (estados_alinha_fase() == 3) return t;
  }
  return -1;
}

int main() {
  memset(&P, 0, sizeof(P));
  cinematica_init(); estados_init(); chave_init(); arena_init();
  P.saude.pronto = true;
  passo(50);

  // 1. oponente a 40 graus na esquerda, fora do campo dos ToF da frente (32 graus)
  reinicia(0, 40);
  estados_alinha(true, 30);
  confere(estados_alinha_ativo() && contem("alinha: ligado por 30 s"), "alinha on 30: liga e avisa no terminal", estados_alinha_restante_s());
  int t1 = ateAlinhar(8000);
  confere(t1 >= 0, "oponente a +40 graus: chega em ALINHADO", t1);
  float err = envolve(beta - psi);
  confere(fabsf(err) <= ALINHA_TOL_DEG + 0.6f, "erro final dentro da tolerância (graus)", err);
  confere(vMax < 5.0f, "só gira: o avanço estimado nunca passa de 5 mm/s", vMax);
  confere(somaPwmMax <= 8, "as duas rodas sempre com PWM opostos (giro no eixo)", somaPwmMax);
  confere(contem("ALINHADO"), "o terminal avisa ALINHADO", 0);
  passo(1500);
  confere(estados_alinha_fase() == 3 && fabsf(envolve(beta - psi)) <= ALINHA_TOL_DEG + 0.6f, "alinhado fica parado e alinhado por 1,5 s", envolve(beta - psi));
  confere(pwmE == 0 && pwmD == 0, "alinhado: rodas paradas", pwmE);

  // 2. oponente atrás (180 graus): busca em pulsos até achar
  reinicia(0, 180);
  estados_alinha(true, 30);
  passo(400);
  confere(estados_alinha_fase() == 1, "oponente atrás: fase 1 (buscando)", estados_alinha_fase());
  int t2 = ateAlinhar(15000);
  confere(t2 >= 0, "oponente atrás: encontra e alinha", t2);
  confere(fabsf(envolve(beta - psi)) <= ALINHA_TOL_DEG + 0.6f, "oponente atrás: erro final dentro da tolerância", envolve(beta - psi));
  confere(vMax < 5.0f, "oponente atrás: nunca andou", vMax);

  // 3. oponente a -100 graus (direita): vai para o lado em que o último visto estava ou, sem pista, a esquerda; ainda assim acha
  reinicia(0, -100);
  estados_alinha(true, 30);
  int t3 = ateAlinhar(15000);
  confere(t3 >= 0 && fabsf(envolve(beta - psi)) <= ALINHA_TOL_DEG + 0.6f, "oponente a -100 graus: alinha", envolve(beta - psi));

  // 4. rodas girando 1,8 vez mais que o modelo: não fica oscilando, termina alinhado no melhor que dá
  reinicia(0, 25, 1.8f);
  estados_alinha(true, 30);
  int t4 = ateAlinhar(10000);
  confere(t4 >= 0, "rodas girando demais: termina em ALINHADO", t4);
  confere(fabsf(envolve(beta - psi)) <= 2.5f * ALINHA_TOL_DEG + 0.6f, "rodas girando demais: erro final pequeno", envolve(beta - psi));
  // e rodas girando metade do que o modelo diz
  reinicia(0, 25, 0.5f);
  estados_alinha(true, 30);
  int t4b = ateAlinhar(15000);
  confere(t4b >= 0 && fabsf(envolve(beta - psi)) <= ALINHA_TOL_DEG + 0.6f, "rodas girando metade: ainda alinha", envolve(beta - psi));

  // 5. a linha (borda) vista durante o alinhamento não para nada
  reinicia(0, 20);
  estados_alinha(true, 30);
  P.borda.detectada = true; P.borda.mascara = 1; P.borda.cru = 1; P.borda.frente = true; P.borda.lado = -1;
  int t5 = ateAlinhar(8000);
  confere(t5 >= 0 && estados_alinha_ativo(), "linha branca vista o tempo todo: alinha do mesmo jeito", t5);
  confere(estado_atual() == ESPERA, "e a máquina de estados continua em ESPERA", estado_atual());
  P.borda = Borda{};

  // 6. alinhado, o oponente anda 15 graus: volta a girar e alinha de novo
  reinicia(0, 10);
  estados_alinha(true, 60);
  ateAlinhar(8000);
  beta += 15;
  passo(1500);
  confere(estados_alinha_fase() == 2 || estados_alinha_fase() == 3, "oponente andou 15 graus: reage", estados_alinha_fase());
  int t6 = ateAlinhar(6000);
  confere((t6 >= 0 || estados_alinha_fase() == 3) && fabsf(envolve(beta - psi)) <= ALINHA_TOL_DEG + 0.6f, "e alinha de novo", envolve(beta - psi));
  // some do campo: depois de ALINHA_PERDE_MS volta a procurar
  oponentePresente = false;
  passo(2500);
  confere(estados_alinha_fase() == 1, "oponente sumiu: volta a buscar", estados_alinha_fase());

  // 7. tempo esgotado
  reinicia(0, 170);
  estados_alinha(true, 2);
  passo(2300);
  confere(!estados_alinha_ativo() && contem("tempo esgotado"), "alinha on 2: termina depois de 2 s", estados_alinha_restante_s());
  confere(pwmE == 0 && pwmD == 0, "e as rodas ficam paradas", pwmE);

  // 8. alinha off, stop e go
  reinicia(0, 170);
  estados_alinha(true, 30); passo(300);
  estados_alinha(false, 0); passo(40);
  confere(!estados_alinha_ativo() && pwmE == 0 && pwmD == 0, "alinha off: para na hora", pwmE);
  reinicia(0, 170);
  estados_alinha(true, 30); passo(300);
  estados_parar(); passo(40);
  confere(!estados_alinha_ativo() && pwmE == 0 && pwmD == 0, "stop: desliga o alinha", pwmE);
  reinicia(0, 170);
  estados_alinha(true, 30); passo(300);
  estados_iniciar(); passo(10);
  confere(!estados_alinha_ativo() && estado_atual() == CONTAGEM, "go: desliga o alinha e começa a contagem", estado_atual());
  estados_parar();

  // 9. só liga em ESPERA
  reinicia(0, 170);
  estados_iniciar(); passo(10);
  texto.clear();
  estados_alinha(true, 30);
  confere(!estados_alinha_ativo() && contem("so funciona com o robo parado"), "alinha em CONTAGEM: recusa e explica", estado_atual());
  estados_parar();

  // 10. mov e pwm: sem flag a linha encerra o teste, com flag 1 não, com flag 2 o terminal fica calado
  reinicia(0, 170);
  estados_teste_mov(200, 0, 1000); P.borda.detectada = true; P.borda.mascara = 1;
  passo(40);
  confere(!estados_teste_ativo() && contem("(BORDA)"), "mov sem flag: a linha encerra o teste", 0);
  reinicia(0, 170);
  P.borda.detectada = true; P.borda.mascara = 1;
  estados_teste_mov(200, 0, 1000, TESTE_IGNORA_BORDA); passo(400);
  confere(estados_teste_ativo() && cinematica_v_mms() > 50 && contem("linha ignorada"), "mov com flag 1: anda com a linha marcando", cinematica_v_mms());
  passo(700);
  confere(!estados_teste_ativo() && !contem("(BORDA)"), "mov com flag 1: só termina pelo tempo", 0);
  reinicia(0, 170);
  P.borda.detectada = true; P.borda.mascara = 1;
  estados_teste_pwm(40, 40, 1000, TESTE_IGNORA_BORDA); passo(200);
  confere(estados_teste_ativo() && pwmE > 80 && pwmD > 80, "pwm com flag 1: roda com a linha marcando", pwmE);
  reinicia(0, 170);
  estados_teste_mov(0, 90, 100, TESTE_IGNORA_BORDA | TESTE_QUIETO); passo(300);
  confere(!contem("# mov"), "flag 2: nenhuma linha # mov no terminal", texto.size());
  P.borda = Borda{};

  // 11. alinha liga o mov desliga: um teste mov pede o motor e desliga o alinha
  reinicia(0, 170);
  estados_alinha(true, 30); passo(100);
  estados_teste_mov(100, 0, 500, TESTE_IGNORA_BORDA); passo(40);
  confere(!estados_alinha_ativo() && estados_teste_ativo(), "mov durante o alinha: o alinha sai da frente", estados_alinha_ativo());
  estados_parar();

  printf(falhas ? "\nERRO: %d falha(s)\n" : "\nOK: 0 falha(s)\n", falhas);
  return falhas ? 1 : 0;
}
