// teste_motores.cpp
// Testes do acionamento da ponte H no computador, sem ESP32. As funções do LEDC abaixo só anotam
// o que o motores.cpp pede, e o teste confere três coisas: cada pino IN recebe o duty certo nos
// dois sentidos, a configuração do PWM acontece só uma vez no motores_init() e um comando
// repetido em toda volta do loop não mexe no periférico. A configuração repetida era o defeito
// do analogWrite no core 2.x: cada chamada zerava o contador do PWM.
// Uso (core 2.x e core 3.x):
//   g++ -std=c++11 -Wall -Wextra -I teste/stub -I . -DUSAR_MOTORES=1 teste/teste_motores.cpp motores.cpp -o t && ./t
//   g++ -std=c++11 -Wall -Wextra -I teste/stub -I . -DUSAR_MOTORES=1 -DESP_ARDUINO_VERSION_MAJOR=3 teste/teste_motores.cpp motores.cpp -o t && ./t
#include <cstdio>
#include "motores.h"
#include "config.h"

uint32_t relogio_us = 0;
size_t SerialFalso::write(uint8_t c) { putchar(c); return 1; }
SerialFalso Serial;

static int duty[40];           // duty atual em cada GPIO
static int configuracoes = 0;  // quantas vezes o PWM foi configurado (timer e canal)
static int escritas = 0;       // quantas escritas de duty chegaram no periférico
static uint32_t freqUsada = 0;
static int bitsUsados = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution) {
  configuracoes++; freqUsada = freq; bitsUsados = resolution; duty[pin] = 0; return true;
}
bool ledcWrite(uint8_t pin, uint32_t d) { escritas++; duty[pin] = (int)d; return true; }
#else
static int pinoDoCanal[16];
uint32_t ledcSetup(uint8_t, uint32_t freq, uint8_t resolution_bits) {
  configuracoes++; freqUsada = freq; bitsUsados = resolution_bits; return freq;
}
void ledcAttachPin(uint8_t pin, uint8_t channel) { pinoDoCanal[channel] = pin; duty[pin] = 0; }
void ledcWrite(uint8_t channel, uint32_t d) { escritas++; duty[pinoDoCanal[channel]] = (int)d; }
#endif

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor) {
  printf("%s  %-62s %10.2f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}
static bool pinos(int in1, int in2, int in3, int in4) {
  return duty[PINO_IN1] == in1 && duty[PINO_IN2] == in2 && duty[PINO_IN3] == in3 && duty[PINO_IN4] == in4;
}

int main() {
  printf("# core %d.x\n", ESP_ARDUINO_VERSION_MAJOR);
  motores_init();
  confere(configuracoes == 4, "init: PWM configurado uma vez em cada um dos 4 pinos", configuracoes);
  confere(freqUsada == PWM_MOTOR_HZ && freqUsada <= 2000, "init: frequencia do PWM dentro do limite do modulo", freqUsada);
  confere(bitsUsados == 8, "init: resolucao de 8 bits (duty de 0 a 255)", bitsUsados);
  confere(pinos(0, 0, 0, 0), "init: os quatro IN em 0 (motores parados)", 0);

  // pwm 30 30: 30% de 255 dá 76 no IN1 e no IN3, com IN2 e IN4 em 0
  motores(76, 76);
  confere(pinos(76, 0, 76, 0), "frente 30%: PWM no IN1 (GPIO 25) e no IN3 (GPIO 27)", duty[PINO_IN3]);

  // o loop repete o mesmo comando milhares de vezes por segundo
  int cfgAntes = configuracoes, escAntes = escritas;
  for (int i = 0; i < 5000; i++) motores(76, 76);
  confere(configuracoes == cfgAntes, "loop: comando repetido nao reconfigura o PWM", configuracoes - cfgAntes);
  confere(escritas == escAntes, "loop: comando repetido nao escreve de novo no periferico", escritas - escAntes);
  confere(pinos(76, 0, 76, 0), "loop: duty continua o mesmo", duty[PINO_IN1]);

  motores(-76, -76);
  confere(pinos(0, 76, 0, 76), "tras 30%: PWM no IN2 (GPIO 26) e no IN4 (GPIO 13)", duty[PINO_IN4]);

  motores(255, -255);
  confere(pinos(255, 0, 0, 255), "giro: esquerda para frente e direita para tras a 100%", duty[PINO_IN1]);

  motores(400, -400);
  confere(pinos(255, 0, 0, 255), "comando fora da faixa limitado a 255", duty[PINO_IN4]);

  motores(0, 76);
  confere(pinos(0, 0, 76, 0), "so a direita: esquerda parada, PWM no IN3", duty[PINO_IN3]);

  motores_para();
  confere(pinos(0, 0, 0, 0), "parada: os quatro IN em 0", 0);
  confere(configuracoes == 4, "fim: PWM configurado so no init", configuracoes);

  printf("\n%s: %d falha(s)\n", falhas ? "ERRO" : "OK", falhas);
  return falhas ? 1 : 0;
}
