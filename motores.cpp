// motores.cpp
// Controle dos dois motores pela ponte H TB6612FNG. Com USAR_MOTORES 0 (bancada), o módulo só
// guarda o comando, e o resto do código funciona igual sem mexer em nenhum pino.

#include "motores.h"
#include "config.h"

static int16_t cmdE = 0, cmdD = 0;   // último comando de cada roda

void motores_init() {
#if USAR_MOTORES
  pinMode(PINO_AIN1, OUTPUT); pinMode(PINO_AIN2, OUTPUT);
  pinMode(PINO_BIN1, OUTPUT); pinMode(PINO_BIN2, OUTPUT);
  pinMode(PINO_PWMA, OUTPUT); pinMode(PINO_PWMB, OUTPUT);
#endif
  motores_para();
}

#if USAR_MOTORES
// Acionamento de um motor: os pinos IN1 e IN2 escolhem o sentido e o PWM define a força.
// Com v igual a 0 os dois pinos ficam em LOW e o motor fica solto.
static void umMotor(int16_t v, uint8_t in1, uint8_t in2, uint8_t pwm) {
  v = constrain(v, -255, 255);
  if (v > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (v < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else            { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
  analogWrite(pwm, abs(v));
}
#endif

// Função principal do módulo. O INVERTE_ESQ e o INVERTE_DIR do config.h corrigem um motor
// soldado ao contrário sem precisar trocar fios.
void motores(int16_t esq, int16_t dir) {
  cmdE = esq; cmdD = dir;
#if USAR_MOTORES
  umMotor(INVERTE_ESQ ? -esq : esq, PINO_AIN1, PINO_AIN2, PINO_PWMA);
  umMotor(INVERTE_DIR ? -dir : dir, PINO_BIN1, PINO_BIN2, PINO_PWMB);
#endif
}

void motores_para() { motores(0, 0); }
int16_t motores_esq() { return cmdE; }
int16_t motores_dir() { return cmdD; }
