// arena.h
// Localização do robô dentro do dojô sem encoder e sem IMU, só com o modelo das rodas
// (cinematica.cpp) e os 4 sensores de borda. O dojô é um círculo, então o rumo absoluto não
// importa: basta saber onde fica o centro do dojô visto do robô (cx, cy, no referencial do
// robô, em mm). O filtro de partículas guarda ARENA_N hipóteses desse ponto [THRUN; BURGARD;
// FOX, 2005]:
//   - predição: a cada 5 ms o centro "anda" ao contrário do movimento estimado do robô, com
//     ruído proporcional ao movimento e muito maior quando o robô está empurrando ou sendo
//     empurrado (nesse caso o modelo das rodas não diz quanto o robô andou de verdade);
//   - correção: cada sensor de borda no branco diz que ele está na faixa (raio de 360 a 385 mm),
//     e cada sensor no preto diz que ele não está nela;
//   - reamostragem quando poucas partículas concentram o peso, com um sorteio pequeno na posição
//     e na escala do modelo das rodas de cada cópia (cada partícula carrega o seu erro de escala).
// Com 600 partículas, o passo leva uns 20 us no PC e deve ficar abaixo de 1 ms no ESP32 (núcleo
// 1, a cada 5 ms). Resultado do teste/unidade/teste_arena.cpp (150 lutas, modelo das rodas certo ou
// errado em 15%): depois de 5 toques na borda, erro mediano do centro de 20 a 25 mm e 90% das
// lutas abaixo de 60 mm.
// Usos: a fuga da borda gira na direção do centro e volta para o meio do dojô; a busca e o
// flanco escolhem o lado com mais espaço; e a percepção descarta leituras de ToF e posições do
// mapa do rastreador que cairiam fora do dojô (pessoas, juiz, parede), para o robô nunca ir
// atrás de algo de fora.
#pragma once
#include <Arduino.h>

#ifndef ARENA_AMOSTRA
#define ARENA_AMOSTRA 64       // partículas copiadas na saída (para o descarte no outro núcleo)
#endif

struct Arena {
  bool  valida;      // filtro iniciado (depois do start)
  bool  confiavel;   // incerteza pequena o bastante para guiar e para descartar leituras
  float cx, cy;      // centro do dojô no referencial do robô (mm)
  float dist;        // distância do robô até o centro (mm)
  float ang;         // direção do centro em graus (0 é frente, positivo é esquerda)
  float sigma;       // incerteza da posição em mm (maior desvio padrão)
  float ax[ARENA_AMOSTRA], ay[ARENA_AMOSTRA];   // amostra das hipóteses do centro (mm)
};

void  arena_init();
// Começo da luta. Com o oponente visto, a regra da CoRA (cada robô na sua metade, atrás da
// linha Shikiri) diz que o centro fica entre os dois robôs, e isso entra na distribuição inicial.
void  arena_inicia(bool opVisto, float opX, float opY);
// Uma chamada por volta do loop: v e w estimados (mm/s e graus/s), borda crua e confirmada e
// se o robô está em contato com o oponente.
void  arena_atualiza(float v_mms, float w_dps, uint16_t bordaCru, uint16_t bordaConf, bool contato);
void  arena_le(Arena& a);           // cópia protegida (pode ser chamada do outro núcleo)
// Distância em mm, a partir do robô e na direção (ux, uy) de comprimento 1, além da qual um ponto
// fica a mais de ARENA_MARGEM_MM fora do dojô em todas as hipóteses da amostra, menos
// ARENA_DISCORDA delas. Sem localização em uso (filtro parado ou incerteza acima de
// ARENA_SIGMA_DESCARTE) a resposta é infinita e nada é descartado. O rastreador usa este limite
// em cada direção do mapa, e o descarte das leituras usa o arena_fora abaixo, com o mesmo critério.
float arena_limite(const Arena& a, float ux, float uy);
// true quando o ponto (x, y) do referencial do robô passa desse limite (leitura de fora do dojô).
// No teste_arena, com o oponente a 6 cm da borda em qualquer direção, nenhuma das 9000 medidas
// foi descartada, e a pessoa a 12 cm da borda foi descartada em cerca de 40% delas (o resto fica
// com o alcance de 600 mm e com a regra da borda de trás).
bool  arena_fora(const Arena& a, float x, float y);
// Tempo do último passo do filtro e o pior desde o start, em microssegundos (comando arena)
uint32_t arena_us();
uint32_t arena_us_max();
// Distância em mm do robô até a borda andando na direção angGraus (rumo livre)
float arena_livre(float angGraus);
