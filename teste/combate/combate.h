// combate.h
// Simulador de combate com dois firmwares iguais no mesmo dojô: o robô do Grupo D (robô 0, "A")
// e o adversário (robô 1, "B"), cada um com o próprio estado, a própria chave DIP e o próprio
// botão de start. A mesma biblioteca serve para a bateria de lutas no computador (main_nativo.cpp)
// e para a página 3D (wasm_api.cpp).
#pragma once
#include <stdint.h>
#include "fisica.h"
#include "ponte.h"

namespace cb {

struct Config {
  uint32_t semente  = 1;
  int      estrA    = 0;       // estratégia do Grupo D: 0 Rush, 1 Flank, 2 Fan out
  int      estrB    = -1;      // estratégia do adversário (-1 sorteada)
  bool     brancoA  = false;   // cor do robô do Grupo D (PLA preto por padrão)
  bool     brancoB  = false;   // cor do adversário
  int      rpmB     = 1000;    // motores do adversário: 1000, 600 ou 319 rpm (319 é igual ao do Grupo D)
  int      posA     = -1;      // rumo do Grupo D na largada: -1 sorteado, 0 de frente para o adversário,
  int      posB     = -1;      // 1 e 2 de lado, 3 de costas (posB: o mesmo para o adversário)
  double   tRound   = 90;      // segundos de luta depois dos 5 s parados
  double   atrito   = 1.0;     // fator no atrito do pneu (fisica.h: 1,15 parado e 1,30 escorregando; 0,8 com pó)
  double   restit   = 0.4;     // coeficiente de restituição do choque entre os corpos
  int      manobraA = -1;      // manobra do empurrão travado do Grupo D (-1: a do config.h)
  int      manobraB = 0;       // a do adversário (0: só detecta, como o firmware sem a manobra)
  bool     pessoas  = true;    // operadores e juiz em volta do dojô
  double   massa    = 0.98;
};

struct Resultado {
  bool   fim = false;
  int    vencedor = -1;        // 0 Grupo D, 1 adversário, -1 empate
  int    motivo = 0;           // 1 empurrado para fora, 2 saiu sozinho, 3 tempo esgotado, 4 os dois caíram
  double tempo = 0;            // segundos de luta (depois dos 5 s)
};

struct Estatistica {
  double tContato = 0;         // segundos com os robôs encostados
  double tLevantado[2] = {0, 0};   // segundos com a rampa do outro tirando mais de 30% da carga das rodas
  double aquecMax[2] = {0, 0};
  int    destravas[2] = {0, 0};
  int    manobras[2] = {0, 0};     // manobras do empurrão travado (antitrava.h)
  double fMaxContato = 0;
  double tPrimContato = -1;
  int    jeito[2] = {0, 0};      // rumo na largada: 0 frente a frente, 1 e 2 de lado, 3 de costas
};

struct Evento { double t; int robo; char texto[96]; };

void          inicia(const Config& c);
bool          passo(double ms);                // avança a simulação; false depois do fim do round
double        agora();                         // segundos desde o boot
double        tLiberado();                     // instante em que os robôs podem andar (-1 antes do start)
const Mundo&  mundoAtual();
const Config& configAtual();
const Resultado& resultado();
const Estatistica& estatistica();
void          leFirmware(int robo, FwLeitura& l);
const FwApi&  api(int robo);
int           eventos(const Evento** lista);   // eventos do Log desde a última chamada
// Controle manual de um robô: com ativo, o PWM de cada roda (-1 a 1) substitui o do firmware
// depois dos 5 s parados daquele robô. O firmware continua rodando, com os sensores e o Log.
void          manual(int robo, bool ativo, double dutyE, double dutyD);
bool          manualAtivo(int robo);
const char*   nomeMotivo(int m);

}  // namespace cb
