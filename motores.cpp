// motores.cpp
// Controle dos dois motores pelo módulo de ponte H dupla de 10 A (entradas IN1 a IN4). Com
// USAR_MOTORES 0 (bancada), o módulo só guarda o comando, e o resto do código funciona igual
// sem mexer em nenhum pino.

#include "motores.h"
#include "config.h"

static int16_t cmdE = 0, cmdD = 0;   // último comando de cada roda

#if USAR_MOTORES
#if TROCA_MOTORES
static const uint8_t E1 = PINO_IN3, E2 = PINO_IN4, D1 = PINO_IN1, D2 = PINO_IN2;
#else
static const uint8_t E1 = PINO_IN1, E2 = PINO_IN2, D1 = PINO_IN3, D2 = PINO_IN4;
#endif

// Acionamento de um motor. O pino do sentido escolhido recebe o PWM e o outro fica em LOW;
// com v igual a 0 os dois pinos ficam em LOW e o motor para. O PWM sai a 1 kHz, a frequência
// padrão do analogWrite do ESP32 nos cores 2.x e 3.x, abaixo do limite de 2 kHz do módulo.
// Os dois pinos usam sempre analogWrite, porque um pino preso ao PWM do ESP32 não volta a
// obedecer ao digitalWrite em todas as versões do core.
static void umMotor(int16_t v, uint8_t inA, uint8_t inB) {
  v = constrain(v, -255, 255);
  if (v >= 0) { analogWrite(inB, 0); analogWrite(inA, v);  }
  else        { analogWrite(inA, 0); analogWrite(inB, -v); }
}
#endif

void motores_init() {
#if USAR_MOTORES
  pinMode(E1, OUTPUT); pinMode(E2, OUTPUT);
  pinMode(D1, OUTPUT); pinMode(D2, OUTPUT);
#endif
  motores_para();
}

// Função principal do módulo. O INVERTE_ESQ e o INVERTE_DIR do config.h corrigem um motor
// soldado ao contrário sem precisar trocar fios.
void motores(int16_t esq, int16_t dir) {
  cmdE = esq; cmdD = dir;
#if USAR_MOTORES
  umMotor(INVERTE_ESQ ? -esq : esq, E1, E2);
  umMotor(INVERTE_DIR ? -dir : dir, D1, D2);
#endif
}

void motores_para() { motores(0, 0); }
int16_t motores_esq() { return cmdE; }
int16_t motores_dir() { return cmdD; }
