// hal.h
// Ligação entre o firmware e o mundo simulado: relógio, pinos, ToF, borda e motores.
#pragma once
#include <stdint.h>
#include "mundo.h"

namespace sim {

struct Hal {
  Mundo*   mundo = nullptr;
  uint64_t us = 0;              // relógio da simulação em microssegundos
  bool     verboso = false;     // Log do firmware no stdout
  uint8_t  dip = 0;             // chave DIP: bit 0 é a DIP 1 (GPIO 12) ... bit 3 é a DIP 4 (GPIO 15)
  bool     botao = false;       // botão de start apertado
  uint16_t tofFalho = 0;        // bit i: ToF i não responde (teste de robustez)
  int      duty[40] = {0};      // duty de 0 a 255 em cada GPIO (ledcWrite)
  int      pinoDoCanal[16] = {0};
  // contadores tirados do Log do firmware
  int      trocasEstado = 0;
  char     ultimaLinha[256] = {0};
  void   (*aoLinha)(const char* linha) = nullptr;   // cada linha completa do Log
};
extern Hal H;

// tensão média em cada motor a partir do duty nos pinos IN (ponte HW-956: PWM no IN do sentido)
double tensaoMotorEsq(double vDisp);
double tensaoMotorDir(double vDisp);

}  // namespace sim
