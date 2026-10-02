// rastreador.h
// Filtro bayesiano em grade para um oponente de tamanho conhecido.
// Em vez de fazer a média dos pontos que os sensores veem, o robô guarda um mapa de
// probabilidade de onde está o centro do oponente, em uma grade polar ao redor do robô
// (120 direções de 3 graus por anéis de 20 mm). A cada 20 ms o mapa passa por 4 etapas:
//   1) predição: o oponente pode ter andado (até OP_VEL_MAX_MMS) e o robô pode ter girado,
//      então o mapa espalha um pouco e gira junto quando o giro vem dos motores;
//   2) correção: cada sensor confirma ou desmente cada posição; uma leitura z confirma as
//      posições em que a face do oponente ficaria a z mm, e a falta de leitura desmente as
//      posições no meio do cone (informação negativa);
//   3) regra: no máximo um oponente, quadrado de 15,2 cm, fora do corpo do robô;
//   4) existência: o oponente pode não estar ali (arena vazia), e a chance de existir só
//      sobe quando as leituras aparecem repetidas e coerentes (2 a 3 medidas).
// Resultado na simulação (oponente preto andando até 1,2 m/s, sensores da bancada, robô
// girado na mão até 200 graus/s), erro de ângulo em mediana | 90% dos casos:
//   frente até 12 graus:  3,0 | 10,3 na fusão simples e 2,7 | 7,5 no rastreador
//   diagonal 12 a 35:     7,3 | 14,0 na fusão simples e 3,4 | 8,6 no rastreador
//   entre FE e LE:       23,3 | 34,2 na fusão simples e 10,7 | 20,3 no rastreador
//   distância do centro: 14 mm contra 10 mm; detecção com o oponente à vista: 88% contra 97%
// Quem usa este módulo é o oponente.cpp (fundeBayes) e o debug.cpp (mapa para o painel).

#pragma once
#include <Arduino.h>
#include "config.h"

// grade polar: 120 direções de 3 graus e anéis de 20 mm, de 150 mm até o alcance
#define RAST_NB  120
#define RAST_NR  ((TOF_ALCANCE_MM + 50) / 20 + 1)

bool  rastreador_init();     // alocação e pré-cálculo das tabelas (uns 75 KB de RAM e 40 ms no boot)
void  rastreador_zera();     // mapa de volta para "pode estar em qualquer lugar"

// z[i]: leitura corrigida do sensor i em mm (-1 sem nada); novo[i]: medida nova desde o último passo
// giro_dps: giro do robô (positivo é esquerda); giroConhecido indica que o valor veio dos motores
// avanco_mms: velocidade do robô para a frente (só aumenta a incerteza da distância)
void  rastreador_passo(const int16_t* z, const bool* novo, float giro_dps, bool giroConhecido,
                       float avanco_mms, float dt);

// Resultado de cada passo do filtro, lido pelo oponente.cpp
struct Estimativa {
  float    angulo;       // graus até o centro do oponente (positivo é esquerda)
  float    rho;          // distância em mm da origem até o centro do oponente
  float    x, y;         // posição do centro do oponente em mm
  float    sigma;        // incerteza do ângulo em graus (um desvio padrão)
  float    massa;        // probabilidade dentro da janela do pico, caso o oponente exista (0 a 1)
  float    existe;       // probabilidade de existir oponente dentro do alcance (0 a 1)
  float    pCentro;      // probabilidade de o centro estar em |ângulo| <= CENTRO_DEG
  uint16_t mascara;      // sensores cuja leitura combina com a estimativa
  int16_t  distSensor;   // menor leitura entre os sensores da máscara em mm (-1 se nenhum)
  float    buscaAng;     // giro em graus que faz os cones varrerem mais probabilidade
                         // (na prática, olhar os pontos cegos onde ele pode estar)
  float    buscaGanho;   // parte do mapa que esse giro vai cobrir (0 a 1)
};
Estimativa rastreador_estimativa();   // última estimativa pronta (cópia protegida entre os núcleos)
// Maior leitura do sensor i que ainda cai dentro do círculo de raio TOF_ALCANCE_MM em volta do
// centro do robô (exemplo: FC na ponta da frente, 542 - 87 = 455 mm). Mais longe vira "nada".
float    rastreador_alcance_sensor(uint8_t i);
bool     rastreador_paralelo();       // true quando o filtro roda em uma tarefa no outro núcleo
uint32_t rastreador_us();       // duração do último passo em microssegundos
uint32_t rastreador_us_max();   // pior passo desde o último rastreador_us_zera()
void     rastreador_us_zera();

// Mapa comprimido para o painel, com MAPA_NB direções e MAPA_NR anéis.
// Cada célula vira um caractere de '0' a 'o' (nível 0 a 63, raiz da probabilidade em relação
// ao pico); "!" seguido de um caractere c indica (c - '0') células seguidas com nível 0.
// A direção m cobre de -180 + 9m até -180 + 9(m+1) graus e o anel r cobre de MAPA_R0 + 40r
// até MAPA_R0 + 40(r+1) mm.
uint16_t rastreador_mapa(char* buf, uint16_t n);
#define MAPA_NB 40
#define MAPA_NR ((RAST_NR + 1) / 2)
#define MAPA_R0 140
#define MAPA_DR 40
