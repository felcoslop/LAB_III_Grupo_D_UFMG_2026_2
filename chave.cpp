// chave.cpp
// Leitura da chave DIP (explicação no chave.h e no config.h).

#include "chave.h"
#include "config.h"

static const uint8_t PINOS[4] = {PINO_DIP1, PINO_DIP2, PINO_DIP3, PINO_DIP4};
static uint8_t  bitsFiltrados = 0, bitsCru = 0;
static uint32_t tMudou = 0;
static int8_t   forcaEstr = -1, forcaEscuro = -1;
static const char* NOMES[N_ESTRATEGIAS] = {"RUSH", "FLANK", "FAN_OUT"};

static uint8_t leCru() {
  uint8_t b = 0;
  for (uint8_t i = 0; i < 4; i++) if (digitalRead(PINOS[i]) == HIGH) b |= (1u << i);
  return b;
}

void chave_init() {
  for (uint8_t i = 0; i < 4; i++) pinMode(PINOS[i], INPUT_PULLDOWN);
  bitsCru = bitsFiltrados = leCru();
  tMudou = millis();
}

// A leitura só vale depois de ficar igual por DIP_DEBOUNCE_MS.
void chave_atualiza() {
  uint8_t b = leCru();
  uint32_t agora = millis();
  if (b != bitsCru) { bitsCru = b; tMudou = agora; }
  else if (b != bitsFiltrados && agora - tMudou >= DIP_DEBOUNCE_MS) bitsFiltrados = b;
}

uint8_t chave_bits() { return bitsFiltrados; }

bool chave_conflito() { return __builtin_popcount(bitsFiltrados & 0x07) > 1; }

// DIP 1 = Rush, DIP 2 = Flank, DIP 3 = Fan out; nenhuma ligada vale Rush. Com mais de uma
// ligada vale a de menor número.
Estrategia chave_estrategia() {
  if (forcaEstr >= 0) return (Estrategia)forcaEstr;
  if (bitsFiltrados & 0x01) return RUSH;
  if (bitsFiltrados & 0x02) return FLANK;
  if (bitsFiltrados & 0x04) return FAN_OUT;
  return RUSH;
}

bool chave_escuro() { return forcaEscuro >= 0 ? forcaEscuro != 0 : (bitsFiltrados & 0x08) != 0; }

const char* estrategia_nome(Estrategia e) { return e < N_ESTRATEGIAS ? NOMES[e] : "?"; }
void chave_forca_estrategia(int8_t e) { forcaEstr = e < N_ESTRATEGIAS ? e : -1; }
void chave_forca_escuro(int8_t escuro) { forcaEscuro = escuro; }
