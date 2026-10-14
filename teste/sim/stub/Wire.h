// Wire.h de mentira do simulador: o barramento I2C sempre responde para os endereços dos ToF
// simulados (o comando scan do debug não é usado aqui).
#pragma once
#include "Arduino.h"

class TwoWire {
public:
  bool    begin(int, int) { return true; }
  void    setClock(uint32_t) {}
  void    setTimeOut(uint16_t) {}
  void    beginTransmission(uint8_t a) { alvo = a; }
  uint8_t endTransmission(bool = true);
private:
  uint8_t alvo = 0;
};
extern TwoWire Wire;
