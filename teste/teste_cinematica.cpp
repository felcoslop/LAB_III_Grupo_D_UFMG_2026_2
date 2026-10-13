// teste_cinematica.cpp
// Testes da cinemática das rodas no computador, sem ESP32. O CI compila este arquivo junto com o
// cinematica.cpp do firmware e roda o programa; qualquer verificação que falhe devolve erro.
// Uso: g++ -std=c++11 -Wall -Wextra -I teste/stub -I . teste/teste_cinematica.cpp cinematica.cpp -o t && ./t
#include <cstdio>
#include <cstdlib>
#include "cinematica.h"

uint32_t relogio_us = 0;
static int16_t pwmE = 0, pwmD = 0;         // último comando que chegou nos motores
void motores(int16_t esq, int16_t dir) { pwmE = esq; pwmD = dir; }
void motores_para() { motores(0, 0); }

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor) {
  printf("%s  %-62s %10.2f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}
// roda o comando por "ms" milissegundos, com uma volta do loop a cada 2 ms
static void roda(float v, float w, int ms, Perfil p = PERFIL_NORMAL) {
  for (int t = 0; t < ms; t += 2) { relogio_us += 2000; cinematica_comanda(v, w, p); }
}
static void parado() { cinematica_init(); roda(0, 0, 400, PERFIL_BORDA); cinematica_zera_odometria(); }

int main() {
  // números do modelo (motor de 319 rpm, roda de 32 mm, 3S e L298N)
  confere(VEL_MAX_MMS > 380 && VEL_MAX_MMS < 440, "velocidade maxima entre 380 e 440 mm/s", VEL_MAX_MMS);
  confere(GIRO_MAX_DPS > 330 && GIRO_MAX_DPS < 380, "giro maximo entre 330 e 380 graus/s", GIRO_MAX_DPS);
  confere(DIST_PARADA_MM < BORDA_FAIXA_MM + BORDA_ATE_RODA_MM, "parada na borda menor que a margem ate as rodas", DIST_PARADA_MM);

  // linha reta na velocidade máxima: PWM 100% nas duas rodas e estimativa chegando lá
  parado(); roda(VEL_MAX_MMS, 0, 400);
  confere(pwmE == 255 && pwmD == 255, "linha reta no maximo: PWM 255 nas duas rodas", pwmE);
  confere(fabsf(cinematica_v_mms() - VEL_MAX_MMS) < 0.03f * VEL_MAX_MMS, "linha reta: estimativa a menos de 3% do maximo", cinematica_v_mms());
  confere(cinematica_dist() > 0.8f * VEL_MAX_MMS * 0.4f && cinematica_dist() < VEL_MAX_MMS * 0.4f, "linha reta: distancia estimada em 0,4 s", cinematica_dist());
  confere(fabsf(cinematica_w_dps()) < 1, "linha reta: giro estimado zero", cinematica_w_dps());

  // giro no eixo: rodas em sentidos opostos, mesmo PWM, e giro estimado igual ao pedido
  parado(); roda(0, 300, 500);
  confere(pwmE == -pwmD && pwmD > 0, "giro para a esquerda: roda direita para frente, esquerda para tras", pwmD);
  confere(fabsf(cinematica_w_dps() - 300) < 12, "giro no eixo: estimativa perto de 300 graus/s", cinematica_w_dps());
  confere(fabsf(cinematica_v_mms()) < 2, "giro no eixo: avanco estimado zero", cinematica_v_mms());

  // prioridade para o giro: avanço máximo com giro de 200 graus/s mantém o giro e corta o avanço
  parado(); roda(VEL_MAX_MMS, 200, 500);
  float vEsperado = VEL_MAX_MMS - 200 * (float)DEG_TO_RAD * BITOLA_MM / 2;
  confere(fabsf(cinematica_w_dps() - 200) < 10, "curva no limite: giro mantido em 200 graus/s", cinematica_w_dps());
  confere(fabsf(cinematica_v_mms() - vEsperado) < 0.05f * VEL_MAX_MMS, "curva no limite: avanco cortado para caber nas rodas", cinematica_v_mms());
  confere(pwmD == 255, "curva no limite: roda de fora no maximo", pwmD);

  // giro maior que o possível: fica no máximo do robô, sem passar de 255
  parado(); roda(0, 2000, 400);
  confere(abs(pwmE) <= 255 && abs(pwmD) <= 255 && pwmD == 255, "giro acima do possivel: PWM limitado a 255", pwmD);

  // partida: o limitador faz o comando subir em rampa, e o feed-forward já põe tensão alta
  parado(); roda(VEL_MAX_MMS, 0, 2);
  confere(pwmE > 150, "partida: feed-forward com tensao alta ja na primeira volta", pwmE);

  // reversão nas manobras normais: a rampa freia antes de inverter (não inverte na hora)
  parado(); roda(VEL_MAX_MMS, 0, 400); roda(-VEL_MAX_MMS, 0, 2);
  confere(pwmE >= 0, "reversao normal: primeira volta ainda nao inverte a roda", pwmE);
  // fuga da borda: tensão toda para trás na primeira volta
  parado(); roda(VEL_MAX_MMS, 0, 400); roda(-VEL_MAX_MMS, 0, 2, PERFIL_BORDA);
  confere(pwmE == -255 && pwmD == -255, "fuga da borda: PWM -255 ja na primeira volta", pwmE);

  // giro de recuperação: 135 graus estimados em menos de 0,6 s a 320 graus/s
  parado();
  int ms = 0;
  while (fabsf(cinematica_giro()) < 135 && ms < 2000) { roda(0, -320, 2); ms += 2; }
  confere(ms < 600, "giro de 135 graus na borda termina em menos de 600 ms", ms);
  confere(cinematica_giro() < 0, "giro para a direita acumula angulo negativo", cinematica_giro());

  // zona morta: comando pequeno ainda tira o motor do lugar, comando quase zero freia
  parado(); roda(5, 0, 40);
  confere(pwmE > 40, "5 mm/s: PWM acima da tensao de partida (kS)", pwmE);
  parado(); roda(1, 0, 40);
  confere(pwmE == 0 && pwmD == 0, "1 mm/s: roda freada", pwmE);

  printf("\n%s: %d falha(s)\n", falhas ? "ERRO" : "OK", falhas);
  return falhas ? 1 : 0;
}
