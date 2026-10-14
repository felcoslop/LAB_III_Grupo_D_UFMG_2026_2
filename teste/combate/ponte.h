// ponte.h
// Funções de cada firmware que o simulador de combate chama. Cada robô tem uma tabela destas,
// montada no ponte_robo.cpp dentro do namespace dele (fa para o Grupo D, fb para o adversário).
#pragma once
#include <stdint.h>

struct FwLeitura {
  int   estado;
  bool  opVisto;
  float opX, opY, opAng;          // estimativa do oponente no referencial do robô (mm, graus)
  bool  arenaValida, arenaConfiavel;
  float arenaCx, arenaCy, arenaSigma;
  int   estrategia;               // 0 Rush, 1 Flank, 2 Fan out
  bool  escuro;
  int   travado;
  int   destravas;
  float aquecimento;              // elevação estimada da temperatura do motor mais quente (K)
  bool  limiteTermico;
  uint16_t bordaMascara;
  uint16_t tofMascara;            // ToF vendo o oponente
};

struct FwApi {
  void        (*init)();
  void        (*loop)();
  void        (*le)(FwLeitura& l);
  const char* (*nomeEstado)(int e);
  int         (*nEstados)();
  const char* (*nomeEstrategia)(int e);
};

extern const FwApi fw_api_a;   // robô do Grupo D
extern const FwApi fw_api_b;   // adversário
