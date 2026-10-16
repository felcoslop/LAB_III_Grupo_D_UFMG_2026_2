// hal.cpp
// Implementação do Arduino.h, Wire.h e VL53L0X.h de mentira em cima do mundo simulado.
#include "Arduino.h"
#include "Wire.h"
#include "VL53L0X.h"
#include "config.h"
#include "hal.h"

namespace sim { Hal H; }
using sim::H;

HardwareSerial Serial;
TwoWire Wire;

// ==================== relógio ====================
uint32_t micros() { return (uint32_t)H.us; }
uint32_t millis() { return (uint32_t)(H.us / 1000); }
void delay(uint32_t ms) { H.us += (uint64_t)ms * 1000; }      // só usado no boot (robô parado)
void delayMicroseconds(uint32_t us) { H.us += us; }
void yield() {}

// ==================== pinos ====================
// o pino XSHUT que acabou de virar entrada (sensor ligando) diz qual ToF vai responder no init
static int xshutLigado = -1;
void pinMode(uint8_t pino, uint8_t modo) {
  if (modo != INPUT_PULLUP) return;
  for (uint8_t i = 0; i < N_TOF; i++) if (TOF[i].xshut == pino) xshutLigado = i;
}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t pino) {
  if (pino == PINO_START) return H.botao ? LOW : HIGH;
  // chave DIP: cada via liga o pino no 3V3 (nível alto com a via ligada)
  const uint8_t dips[4] = {12, 14, 2, 15};
  for (int i = 0; i < 4; i++) if (pino == dips[i]) return (H.dip >> i) & 1 ? HIGH : LOW;
  return HIGH;
}
void analogReadResolution(uint8_t) {}
uint32_t analogReadMilliVolts(uint8_t pino) {
  for (uint8_t i = 0; i < N_BORDA; i++)
    if (BORDA[i].pino == pino) return (uint32_t)H.mundo->bordaMv(BORDA[i].x, BORDA[i].y);
  return 0;
}
uint16_t analogRead(uint8_t pino) { return (uint16_t)(analogReadMilliVolts(pino) * 4095 / 3300); }

// ==================== PWM (LEDC) ====================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
bool ledcAttach(uint8_t pin, uint32_t, uint8_t) { H.duty[pin] = 0; return true; }
bool ledcWrite(uint8_t pin, uint32_t duty) { H.duty[pin] = (int)duty; return true; }
#else
uint32_t ledcSetup(uint8_t, uint32_t freq, uint8_t) { return freq; }
void ledcAttachPin(uint8_t pin, uint8_t ch) { H.pinoDoCanal[ch] = pin; H.duty[pin] = 0; }
void ledcWrite(uint8_t ch, uint32_t duty) { H.duty[H.pinoDoCanal[ch]] = (int)duty; }
#endif

namespace sim {
static double frac(int d) { return d >= 256 ? 1.0 : d / 255.0; }   // 256 é o "tudo ligado" do LEDC
double tensaoMotorEsq(double vDisp) {
  int a = TROCA_MOTORES ? PINO_IN3 : PINO_IN1, b = TROCA_MOTORES ? PINO_IN4 : PINO_IN2;
  double v = (frac(H.duty[a]) - frac(H.duty[b])) * vDisp;
  return INVERTE_ESQ ? -v : v;
}
double tensaoMotorDir(double vDisp) {
  int a = TROCA_MOTORES ? PINO_IN1 : PINO_IN3, b = TROCA_MOTORES ? PINO_IN2 : PINO_IN4;
  double v = (frac(H.duty[a]) - frac(H.duty[b])) * vDisp;
  return INVERTE_DIR ? -v : v;
}
}  // namespace sim

// ==================== Serial (Log do firmware) ====================
static char linhaSerial[256];
static int  nSerial = 0;
int HardwareSerial::available() { return 0; }
int HardwareSerial::read() { return -1; }
size_t HardwareSerial::write(uint8_t c) {
  if (H.verboso) { if (c == '\n') ::printf("  [%8.3f s]\n", H.us * 1e-6); else ::putchar(c); }
  if (c == '\n') {
    linhaSerial[nSerial] = 0;
    if (strstr(linhaSerial, "# estado: ")) H.trocasEstado++;
    memcpy(H.ultimaLinha, linhaSerial, sizeof(H.ultimaLinha));
    if (H.aoLinha) H.aoLinha(linhaSerial);
    nSerial = 0;
  } else if (nSerial < (int)sizeof(linhaSerial) - 1) linhaSerial[nSerial++] = (char)c;
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
  // o sensor recém-ligado responde no 0x29; o firmware liga um de cada vez pelo XSHUT, então o
  // último XSHUT que virou entrada diz qual sensor está respondendo. Sensor falho não liga.
  if (xshutLigado >= 0 && (H.tofFalho >> xshutLigado) & 1) return false;
  sinal = 0.25f; vcselPre = 14; vcselFinal = 10; orcamento = 33000; continuo = false;
  return true;
}

void VL53L0X::startContinuous(uint32_t) {
  continuo = true;
  pronta = false;
  proxima = (uint32_t)H.us + orcamento + (uint32_t)H.mundo->uniforme(0, 2000);
}

void VL53L0X::atualiza() {
  if (!continuo) return;
  int i = indiceToF(endereco);
  if (i < 0) return;
  while ((int32_t)((uint32_t)H.us - proxima) >= 0) {
    bool longo = sinal <= 0.15f || vcselPre >= 18;
    // limiar de sinal: preto fosco (refletância 0,05) some perto de 550 mm no modo padrão,
    // como medido na bancada (ALCANCE_PRETO_MM); o perfil de longo alcance enxerga ~30% mais
    // longe, com mais ruído e mais leitura falsa
    sim::PerfilToF pf = longo ? sim::PerfilToF{0.00165 / 1.7, 1.5, 0.008, 2000}
                              : sim::PerfilToF{0.00165, 1.0, 0.003, 1200};
    valor = (uint16_t)H.mundo->tofMede(TOF[i].x, TOF[i].y, TOF[i].ang, pf, nullptr);
    pronta = true;
    proxima += orcamento + 1500;           // orçamento mais o tempo de troca de dados do chip
  }
}

void VL53L0X::writeReg(uint8_t reg, uint8_t) { if (reg == SYSTEM_INTERRUPT_CLEAR) pronta = false; last_status = 0; }

uint8_t VL53L0X::readReg(uint8_t reg) {
  int i = indiceToF(endereco);
  last_status = (i >= 0 && (H.tofFalho >> i) & 1) ? 2 : 0;    // 2: endereço sem resposta
  if (last_status) return 0;
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
