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
  uint16_t bordaMascara;          // sensores de borda confirmados no branco
  uint16_t bordaCru;              // sensores de borda lendo branco agora, ainda sem confirmação
  uint16_t tofMascara;            // ToF vendo o oponente
  int16_t  tof[8];                // leitura de cada ToF em mm (-1 sem nada), como no painel
  float    opExiste, opConfianca; // rastreador: chance de existir oponente e de ele estar na estimativa
  int      antiTrava;             // manobras do empurrão travado nesta luta (antitrava.h)
};

struct FwApi {
  void        (*init)();
  void        (*loop)();
  void        (*le)(FwLeitura& l);
  const char* (*nomeEstado)(int e);
  int         (*nEstados)();
  const char* (*nomeEstrategia)(int e);
  uint16_t    (*mapa)(char* buf, uint16_t n);   // mapa comprimido do rastreador, o mesmo do painel
  int         mapaNb, mapaNr, mapaR0, mapaDr;   // direções, anéis, raio do primeiro anel e largura (mm)
  void        (*manobra)(int m);                // manobra do empurrão travado (-1 deixa a do config.h)
};

extern const FwApi fw_api_a;   // robô do Grupo D
extern const FwApi fw_api_b;   // adversário
