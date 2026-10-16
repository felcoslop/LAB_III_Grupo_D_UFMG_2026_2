// antitrava.h
// Empurrão travado pela caixa do rastreador bayesiano e manobras para derrubar o oponente
// (módulo experimental, fácil de tirar).
//
// Detecção, sem encoder: a caixa do oponente (P.op.x e P.op.y, saída do rastreador.cpp) fica
// parada no referencial do robô, com a distância dentro de ANTI_TRAVA_TOL_MM e o lado (ângulo)
// dentro de ANTI_TRAVA_TOL_DEG, por mais de ANTI_TRAVA_T_MS, com o robô no ATAQUE (PWM alto para
// a frente) e o oponente encostado ou a menos de ANTI_TRAVA_PERTO_MM do contato. Um empurrão que
// funciona também deixa a caixa parada, mas leva o oponente para fora do dojô de 77 cm em pouco
// mais de 1 s; passado esse tempo, os dois robôs estão empatados na força.
//
// Manobras (ANTI_TRAVA_MANOBRA no config.h, ou o comando "trava N" no terminal):
//   0 nenhuma: só detecta e avisa no log;
//   1 lateral: ré em arco curto para o lado com mais espaço e volta em arco pela lateral do
//     oponente (fugir e acertar pela lateral);
//   2 torque: continua empurrando com o giro para o lado com mais espaço (uma roda com tudo e a
//     outra com menos), para a frente do oponente escorregar na rampa e mostrar a quina;
//   3 zigue: o mesmo do 2, alternando o lado a cada ANTI_TRAVA_ZIGUE_MS (os dois lados);
//   4 giro: giro no eixo (giro strike) para o lado com mais espaço, com tensão toda desde o primeiro
//     milissegundo (perfil da borda). Com o oponente empurrando, o robô gira só uns 10 graus antes de ser
//     levado para trás: sem avanço, as duas rodas não resistem ao empurrão;
//   5 pulso: solta as rodas por ANTI_TRAVA_SOLTA_MS e volta com tudo, três vezes (o oponente
//     empurra o robô para trás, o peso vai para a rampa do robô e a ponta dele desce, e o
//     oponente que acelera levanta a própria frente).
//
// Para tirar: ANTI_TRAVA 0 no config.h (este módulo fica vazio e o estados.cpp não chama nada).
// Para apagar de vez: os arquivos antitrava.h e antitrava.cpp, as linhas marcadas com
// ANTI_TRAVA no estados.cpp e no debug.cpp e o nome antitrava nas listas de compilação
// (teste/sim/compila.sh, teste/combate/compila.sh e .github/workflows/compilacao.yml).
#pragma once
#include <Arduino.h>
#include "config.h"

#if ANTI_TRAVA
enum ManobraTrava : uint8_t { MT_NENHUMA, MT_LATERAL, MT_TORQUE, MT_ZIGUE, MT_GIRO, MT_PULSO, MT_N };

void        antitrava_inicia_luta();
// Uma chamada por volta do loop, nos estados de luta, depois da borda. podeAgir: o estado
// atual aceita a manobra (ATAQUE). Devolve true quando a manobra comandou as rodas nesta volta
// (a máquina de estados não comanda mais nada).
bool        antitrava_passo(bool podeAgir);
void        antitrava_aborta();          // borda vista: a fuga vem antes de tudo
bool        antitrava_ativa();
uint8_t     antitrava_manobra();
void        antitrava_define(uint8_t m); // comando "trava N"
const char* antitrava_nome(uint8_t m);
uint16_t    antitrava_disparos();        // manobras nesta luta
uint32_t    antitrava_parado_ms();       // tempo com a caixa parada no ataque
#endif
