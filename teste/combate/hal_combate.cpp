// hal_combate.cpp
// Arduino.h, Wire.h e VL53L0X.h de mentira para dois firmwares no mesmo mundo. É o hal.cpp do
// simulador de luta (teste/sim) com o estado separado por robô: cada chamada atende o robô
// cb::atual, e os dois usam a mesma tabela de sensores do config.h, porque rodam o mesmo firmware.
#include "Arduino.h"
#include "Wire.h"
#include "VL53L0X.h"
#include "config.h"
#include "hal_combate.h"

namespace cb {
HalRobo  HR[2];
int      atual = 0;
uint64_t us = 0;
Mundo*   mundo = nullptr;
bool     verboso = false;
void   (*aoLinha)(int, const char*) = nullptr;

void halReinicia() {
  for (auto& h : HR) h = HalRobo();
  atual = 0;
  us = 0;
}
}  // namespace cb
using cb::HR;
using cb::atual;

HardwareSerial Serial;
TwoWire Wire;

// ==================== relógio ====================
uint32_t micros() { return (uint32_t)cb::us; }
uint32_t millis() { return (uint32_t)(cb::us / 1000); }
void delay(uint32_t ms) { cb::us += (uint64_t)ms * 1000; }     // só no boot, com os robôs parados
void delayMicroseconds(uint32_t v) { cb::us += v; }
void yield() {}

// ==================== pinos ====================
void pinMode(uint8_t pino, uint8_t modo) {
  if (modo != INPUT_PULLUP) return;
  for (uint8_t i = 0; i < N_TOF; i++) if (TOF[i].xshut == pino) HR[atual].xshutLigado = i;
}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t pino) {
  if (pino == PINO_START) return HR[atual].botao ? LOW : HIGH;
  const uint8_t dips[4] = {12, 14, 2, 15};
  for (int i = 0; i < 4; i++) if (pino == dips[i]) return (HR[atual].dip >> i) & 1 ? HIGH : LOW;
  return HIGH;
}
void analogReadResolution(uint8_t) {}
uint32_t analogReadMilliVolts(uint8_t pino) {
  for (uint8_t i = 0; i < N_BORDA; i++)
    if (BORDA[i].pino == pino) return (uint32_t)cb::mundo->bordaMv(atual, BORDA[i].x, BORDA[i].y);
  return 0;
}
uint16_t analogRead(uint8_t pino) { return (uint16_t)(analogReadMilliVolts(pino) * 4095 / 3300); }

// ==================== PWM (LEDC) ====================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
bool ledcAttach(uint8_t pin, uint32_t, uint8_t) { HR[atual].duty[pin] = 0; return true; }
bool ledcWrite(uint8_t pin, uint32_t duty) { HR[atual].duty[pin] = (int)duty; return true; }
#else
uint32_t ledcSetup(uint8_t, uint32_t freq, uint8_t) { return freq; }
void ledcAttachPin(uint8_t pin, uint8_t ch) { HR[atual].pinoDoCanal[ch] = pin; HR[atual].duty[pin] = 0; }
void ledcWrite(uint8_t ch, uint32_t duty) { HR[atual].duty[HR[atual].pinoDoCanal[ch]] = (int)duty; }
#endif

namespace cb {
static double frac(int d) { return d >= 256 ? 1.0 : d / 255.0; }
double dutyMotor(int robo, bool esquerdo) {
  bool a1 = esquerdo != (TROCA_MOTORES != 0);
  int a = a1 ? PINO_IN1 : PINO_IN3, b = a1 ? PINO_IN2 : PINO_IN4;
  double v = frac(HR[robo].duty[a]) - frac(HR[robo].duty[b]);
  bool inv = esquerdo ? INVERTE_ESQ : INVERTE_DIR;
  return inv ? -v : v;
}
}  // namespace cb

// ==================== Serial (Log dos firmwares) ====================
int HardwareSerial::available() { return 0; }
int HardwareSerial::read() { return -1; }
size_t HardwareSerial::write(uint8_t c) {
  cb::HalRobo& h = HR[atual];
  if (c == '\n') {
    h.linha[h.nLinha] = 0;
    if (strstr(h.linha, "# estado: ")) h.trocasEstado++;
    if (cb::verboso) ::printf("[%c %8.3f] %s\n", atual ? 'B' : 'A', cb::us * 1e-6, h.linha);
    if (cb::aoLinha) cb::aoLinha(atual, h.linha);
    h.nLinha = 0;
  } else if (h.nLinha < (int)sizeof(h.linha) - 1) h.linha[h.nLinha++] = (char)c;
  return 1;
}

// ==================== I2C ====================
uint8_t TwoWire::endTransmission(bool) { return 0; }

// ==================== VL53L0X ====================
static int indiceToF(uint8_t endereco) {
  for (uint8_t i = 0; i < N_TOF; i++) if (TOF[i].endereco == endereco) return i;
  return -1;
}

bool VL53L0X::init(bool) {
  sinal = 0.25f; vcselPre = 14; vcselFinal = 10; orcamento = 33000; continuo = false;
  return true;
}

void VL53L0X::startContinuous(uint32_t) {
  continuo = true;
  pronta = false;
  proxima = (uint32_t)cb::us + orcamento + (uint32_t)cb::mundo->uniforme(0, 2000);
}

void VL53L0X::atualiza() {
  if (!continuo) return;
  int i = indiceToF(endereco);
  if (i < 0) return;
  while ((int32_t)((uint32_t)cb::us - proxima) >= 0) {
    bool longo = sinal <= 0.15f || vcselPre >= 18;
    // modelo do teste/sim: preto fosco some perto de 550 mm no perfil padrão (bancada) e o
    // perfil de longo alcance enxerga uns 30% mais longe, com mais ruído. As leituras falsas
    // caem pela metade, porque a sala à noite, só com a lâmpada de LED, quase não tem
    // infravermelho de fundo
    cb::PerfilToF pf = longo ? cb::PerfilToF{0.00165 / 1.7, 1.5, 0.004, 2000}
                             : cb::PerfilToF{0.00165, 1.0, 0.0015, 1200};
    valor = (uint16_t)cb::mundo->tofMede(atual, TOF[i].x, TOF[i].y, TOF[i].ang, pf);
    pronta = true;
    proxima += orcamento + 1500;
  }
}

void VL53L0X::writeReg(uint8_t reg, uint8_t) { if (reg == SYSTEM_INTERRUPT_CLEAR) pronta = false; last_status = 0; }

uint8_t VL53L0X::readReg(uint8_t reg) {
  last_status = 0;
  atualiza();
  if (reg == RESULT_INTERRUPT_STATUS) return pronta ? 0x07 : 0x00;
  return 0;
}

uint16_t VL53L0X::readReg16Bit(uint8_t reg) {
  last_status = 0;
  atualiza();
  if (reg == RESULT_RANGE_STATUS + 10) return valor;
  return 0;
}

uint16_t VL53L0X::readRangeContinuousMillimeters() {
  atualiza();
  pronta = false;
  return valor;
}
