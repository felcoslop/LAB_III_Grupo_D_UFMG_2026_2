// motores.cpp
// Controle dos dois motores pelo módulo HW-956, ponte H dupla de 10 A (entradas IN1 a IN4). Com
// USAR_MOTORES 0 (bancada), o módulo só guarda o comando, e o resto do código funciona igual
// sem mexer em nenhum pino.
//
// O PWM sai do periférico LEDC do ESP32, configurado uma vez só no motores_init(). Depois disso
// cada mudança de comando só troca o duty do canal. O analogWrite não serve aqui: no core 2.x
// cada chamada dele refaz a configuração do timer do LEDC, e essa configuração zera o contador
// do PWM. O loop deste firmware roda em poucas dezenas de microssegundos (sem sensores, menos
// ainda), bem menos que o período de 1 ms do PWM, e com o contador zerado em toda volta o sinal
// no pino deixa de seguir o comando (o duty novo nem chega a valer em alguns ciclos).

#include "motores.h"
#include "config.h"

static_assert(PWM_MOTOR_HZ >= 100 && PWM_MOTOR_HZ <= 2000,
              "PWM_MOTOR_HZ fora da faixa do modulo HW-956 (ate 2 kHz)");

static int16_t cmdE = 0, cmdD = 0;   // último comando de cada roda

#if USAR_MOTORES
#if TROCA_MOTORES
static const uint8_t PINO[4] = { PINO_IN3, PINO_IN4, PINO_IN1, PINO_IN2 };
#else
static const uint8_t PINO[4] = { PINO_IN1, PINO_IN2, PINO_IN3, PINO_IN4 };
#endif
enum { E1, E2, D1, D2 };             // posição de cada entrada no vetor PINO
static int16_t dutyAtual[4] = { -1, -1, -1, -1 };   // -1 obriga a primeira escrita

// A pwmLiga() prende o pino ao LEDC com 8 bits (duty de 0 a 255). No core 2.x o canal fica
// definido aqui (0 a 3, dois por timer, todos na mesma frequência); no core 3.x o próprio core
// escolhe o canal.
static void pwmLiga(uint8_t i) {
  pinMode(PINO[i], OUTPUT);
  digitalWrite(PINO[i], LOW);        // ponte desligada desde o primeiro instante
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttach(PINO[i], PWM_MOTOR_HZ, 8))
    Serial.printf("! motores: falha ao ligar o PWM no GPIO %d\n", PINO[i]);
#else
  ledcSetup(i, PWM_MOTOR_HZ, 8);
  ledcAttachPin(PINO[i], i);
#endif
}

// A pwmEscreve() só escreve o duty quando ele muda, para o loop não mexer no periférico à toa.
static void pwmEscreve(uint8_t i, int16_t duty) {
  if (duty == dutyAtual[i]) return;
  dutyAtual[i] = duty;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PINO[i], duty);
#else
  ledcWrite(i, duty);
#endif
}

// Acionamento de um motor. O pino do sentido escolhido recebe o PWM e o outro fica em LOW;
// com v igual a 0 os dois pinos ficam em LOW e o motor para.
static void umMotor(int16_t v, uint8_t a, uint8_t b) {
  v = constrain(v, -255, 255);
  if (v >= 0) { pwmEscreve(b, 0); pwmEscreve(a, v);  }
  else        { pwmEscreve(a, 0); pwmEscreve(b, -v); }
}
#endif

void motores_init() {
#if USAR_MOTORES
  for (uint8_t i = 0; i < 4; i++) pwmLiga(i);
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
