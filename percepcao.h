// percepcao.h
// Este arquivo define a struct P, que funciona como o "contrato" entre os sensores e a
// máquina de estados. A cada volta do loop a percepção monta uma foto do mundo dentro de P,
// e a máquina de estados (estados.cpp) lê somente essa foto. Com essa separação, a estratégia
// não depende de VL53L0X, I2C ou ADC, e um sensor novo entra sem mexer na estratégia.
//
// Referencial usado por todos os sensores: a origem fica no centro de giro do robô (meio do
// eixo das rodas de tração), x aponta para a frente, y para a esquerda e o ângulo positivo
// também é para a esquerda.

#pragma once
#include <Arduino.h>

// Informação sobre o oponente, preenchida pelo oponente.cpp (com ajuda do rastreador.cpp).
struct Oponente {
  bool     visto;          // oponente confirmado neste momento
  bool     centrado;       // sensor central vendo e |angulo| <= CENTRO_DEG
  float    angulo;         // graus até o centro do oponente: 0 é frente, positivo é esquerda
                           // com o rastreador esse valor continua sendo a melhor estimativa mesmo sem ver
  int16_t  dist_mm;        // menor leitura entre os sensores que veem o alvo (-1 quando nenhum vê)
  uint16_t mascara;        // bit i ligado quando o sensor TOF[i] está vendo o alvo
  int8_t   lado_ultimo;    // +1 provável à esquerda, -1 provável à direita, 0 nunca visto
  uint32_t ms_sem_ver;     // tempo desde a última confirmação de algum sensor (0 se está vendo)
  float    x, y;           // posição do centro do oponente em mm, no referencial do robô
  float    sigma;          // incerteza do ângulo em graus (vale 0 na fusão simples)
  float    confianca;      // de 0 a 1: chance de o oponente estar perto da estimativa
  float    existe;         // de 0 a 1: chance de existir oponente no alcance (arena vazia fica perto de 0)
  float    busca_ang;      // sem ver o oponente: giro em graus que leva os sensores aos pontos cegos
  float    busca_ganho;    // de 0 a 1: parte do mapa que esse giro consegue cobrir
};

// Rumo é a direção que o robô deve seguir agora, e corresponde à seta do painel.
// O cálculo fica no percepcao.cpp e o uso principal é no estado BUSCA do estados.cpp.
enum RumoModo : uint8_t {
  RUMO_NADA = 0,       // ainda sem informação
  RUMO_ATACAR,         // oponente visto e de frente: o robô avança
  RUMO_MIRAR,          // oponente visto fora do centro: o robô gira ou curva até o ângulo zerar
  RUMO_PROVAVEL,       // oponente sumiu há pouco, mas o mapa ainda sabe onde ele deve estar
  RUMO_PROCURAR        // sem pista: o robô gira para varrer os pontos cegos
};
struct Rumo {
  RumoModo modo;
  float    angulo;         // graus para girar (positivo é esquerda), até o oponente ou da busca
  float    dist_mm;        // distância até o centro do oponente (0 durante a procura)
};

// Informação dos sensores de borda, preenchida pelo borda.cpp.
struct Borda {
  bool     detectada;      // alguma borda já confirmada pelo filtro de tempo
  uint16_t mascara;        // bit i ligado quando BORDA[i] está confirmado no branco
  uint16_t cru;            // bit i ligado quando BORDA[i] lê branco agora, ainda sem confirmação
  int8_t   lado;           // -1 esquerda, +1 direita, 0 os dois lados
  bool     frente;         // algum sensor da frente confirmado
  bool     tras;           // algum sensor de trás confirmado
};

// Saúde dos sensores e do ciclo, usada para impedir a luta com sensor faltando.
struct Saude {
  uint8_t  tof_ok;         // quantidade de ToF respondendo
  uint8_t  tof_total;
  uint16_t tof_mascara;    // bit i ligado quando TOF[i] está OK
  bool     pronto;         // existe sensor suficiente para lutar
  uint32_t ciclo_us;       // duração da última volta do loop
  uint32_t ciclo_us_max;   // pior volta desde o último comando "tempo"
};

// A foto completa do mundo em um instante.
struct Percepcao {
  uint32_t t_ms;           // millis() do momento da foto
  Oponente op;
  Rumo     rumo;           // direção a seguir (oponente ou busca)
  Borda    borda;
  Saude    saude;
  bool     start;          // botão de início apertado (evento que dura uma volta do loop)
};

extern Percepcao P;      // a variável fica no percepcao.cpp e é lida por estados.cpp e debug.cpp

void percepcao_init();
void percepcao_atualiza();     // uma chamada por volta do loop, antes da máquina de estados
void percepcao_manutencao();   // função usada só com o robô parado, para religar sensores que falharam
