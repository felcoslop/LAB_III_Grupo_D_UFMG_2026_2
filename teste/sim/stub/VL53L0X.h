// VL53L0X.h de mentira do simulador, com a mesma interface da biblioteca da Pololu (1.3.1) na
// parte que o firmware usa. Cada objeto é um ToF do robô simulado: o endereço que o firmware
// define com setAddress diz qual sensor da tabela TOF[] ele é. No modo contínuo, uma medida
// nova fica pronta a cada orçamento de tempo (setMeasurementTimingBudget) e o valor vem do
// mundo simulado (hal.cpp), que lança raios no cone do sensor contra o oponente e contra os
// objetos fora do dojô. O perfil de sinal (setSignalRateLimit e setVcselPulsePeriod) muda o
// alcance em alvo escuro e o ruído, como no chip de verdade.
#pragma once
#include "Arduino.h"
#include "Wire.h"

class VL53L0X {
public:
  enum regAddr {
    SYSTEM_INTERRUPT_CLEAR  = 0x0B,
    RESULT_INTERRUPT_STATUS = 0x13,
    RESULT_RANGE_STATUS     = 0x14,
  };
  enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };
  uint8_t last_status = 0;

  void     setBus(TwoWire*) {}
  void     setAddress(uint8_t a) { endereco = a; }
  uint8_t  getAddress() { return endereco; }
  bool     init(bool io_2v8 = true);
  void     writeReg(uint8_t reg, uint8_t value);
  uint8_t  readReg(uint8_t reg);
  uint16_t readReg16Bit(uint8_t reg);
  bool     setSignalRateLimit(float mcps) { sinal = mcps; return true; }
  float    getSignalRateLimit() { return sinal; }
  bool     setMeasurementTimingBudget(uint32_t us) { orcamento = us; return true; }
  uint32_t getMeasurementTimingBudget() { return orcamento; }
  bool     setVcselPulsePeriod(vcselPeriodType t, uint8_t p) { if (t == VcselPeriodPreRange) vcselPre = p; else vcselFinal = p; return true; }
  uint8_t  getVcselPulsePeriod(vcselPeriodType t) { return t == VcselPeriodPreRange ? vcselPre : vcselFinal; }
  void     startContinuous(uint32_t = 0);
  void     stopContinuous() { continuo = false; }
  uint16_t readRangeContinuousMillimeters();
  void     setTimeout(uint16_t) {}
  bool     timeoutOccurred() { return false; }

private:
  void     atualiza();
  uint8_t  endereco = 0x29;
  float    sinal = 0.25f;
  uint32_t orcamento = 33000;
  uint8_t  vcselPre = 14, vcselFinal = 10;
  bool     continuo = false;
  uint32_t proxima = 0;          // instante (us) da próxima medida pronta
  bool     pronta = false;
  uint16_t valor = 8190;
};
