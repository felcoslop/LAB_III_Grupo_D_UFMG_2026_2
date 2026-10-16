// hal_combate.h
// Ligação entre os dois firmwares e o mundo do combate. O Arduino de mentira (relógio, pinos,
// chave DIP, botão, sensores de borda, ToF, PWM e Serial) atende o robô da vez: antes de chamar
// o firmware de um robô, o simulador troca cb::atual para 0 (Grupo D) ou 1 (adversário).
#pragma once
#include <stdint.h>
#include "fisica.h"

namespace cb {

struct HalRobo {
  uint8_t  dip = 0;             // bit 0 = DIP 1 ... bit 3 = DIP 4
  bool     botao = false;
  int      duty[40] = {0};      // duty de 0 a 256 em cada GPIO
  int      pinoDoCanal[16] = {0};
  int      xshutLigado = -1;
  char     linha[256] = {0};
  int      nLinha = 0;
  int      trocasEstado = 0;
};

extern HalRobo HR[2];
extern int      atual;          // robô cujo firmware está rodando agora
extern uint64_t us;             // relógio da simulação (us), o mesmo para os dois
extern Mundo*   mundo;
extern bool     verboso;
extern void   (*aoLinha)(int robo, const char* linha);   // cada linha do Log de cada firmware

void   halReinicia();
double dutyMotor(int robo, bool esquerdo);   // -1 a 1, pelo PWM nos pinos IN

}  // namespace cb
