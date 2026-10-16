// percepcao.cpp
// Este módulo junta todos os sensores em uma foto só, a struct P declarada no percepcao.h.
// A cada volta do loop ele lê a chave DIP, pega a estimativa do oponente (oponente.cpp, que
// roda a leitura dos ToF e o rastreador no núcleo 0), calcula o rumo, lê a linha branca
// (borda.cpp), atualiza a localização no dojô (arena.cpp) e a saúde dos sensores. Um sensor
// novo no futuro (IMU, encoder) entra criando o módulo dele e preenchendo um campo novo de P aqui.

#include "percepcao.h"
#include "config.h"
#include "telemetria.h"
#include "oponente.h"
#include "borda.h"
#include "arena.h"
#include "chave.h"
#include "cinematica.h"
#include <Wire.h>

Percepcao P;   // foto do mundo, lida pela máquina de estados (estados.cpp) e pelo debug.cpp

static bool     startAnt = false;   // último estado do botão de start
static uint32_t tStartMud = 0;      // momento da última mudança do botão (debounce)
static uint32_t tCicloAnt = 0;      // início da volta anterior do loop, para medir o ciclo
static uint32_t tContato = 0;       // última vez em que o oponente estava colado na frente
static bool     teveContato = false;
static uint32_t tBordaSem = 0;      // última vez sem nenhuma borda confirmada

// O rumo é a seta do painel. Com o oponente visto, a seta aponta para ele; com o oponente
// perdido há pouco, aponta para onde o mapa diz que ele está; sem pista, aponta para o giro
// que faz os cones dos sensores varrerem os pontos cegos (valor calculado no rastreador.cpp).
static void atualizaRumo() {
  const Oponente& o = P.op;
  Rumo& r = P.rumo;
  r.dist_mm = sqrtf(o.x * o.x + o.y * o.y);
  if (o.visto)                                              { r.modo = o.centrado ? RUMO_ATACAR : RUMO_MIRAR; r.angulo = o.angulo; }
  else if (o.existe >= 0.3f && o.confianca >= 0.15f)        { r.modo = RUMO_PROVAVEL; r.angulo = o.angulo; }
  else {
    r.modo = RUMO_PROCURAR;
    r.dist_mm = 0;
    // quando o mapa não tem pista nenhuma (o giro quase não cobre nada novo), o robô
    // continua girando para o último lado em que o oponente apareceu
    if (o.busca_ganho >= 0.05f) r.angulo = o.busca_ang;
    else                        r.angulo = 90.0f * (o.lado_ultimo != 0 ? o.lado_ultimo : 1);
  }
}

// A saúde conta quantos ToF respondem; com menos de TOF_MINIMO_PARA_LUTAR o robô não inicia.
static void atualizaSaude() {
  P.saude.tof_total   = N_TOF;
  P.saude.tof_mascara = tof_mascara_ok();
  P.saude.tof_ok      = __builtin_popcount(P.saude.tof_mascara);   // contagem dos bits ligados
  P.saude.pronto      = P.saude.tof_ok >= TOF_MINIMO_PARA_LUTAR;
}

void percepcao_init() {
  memset(&P, 0, sizeof(P));
  pinMode(PINO_START, INPUT_PULLUP);
  chave_init();                        // chave DIP: estratégia e perfil do oponente (chave.cpp)
  arena_init();                        // a localização começa no start (estados.cpp)

  Wire.begin(PINO_SDA, PINO_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(20);                 // timeout curto: um sensor travado não trava o robô

  uint8_t n = tof_init();              // os ToF são ligados um por um (oponente.cpp)
  Log.printf("# ToF: %u de %u sensores OK\n", n, (unsigned)N_TOF);
  borda_init();                        // leitura da calibração da borda (borda.cpp)
  atualizaSaude();
  tCicloAnt = micros();
}

void percepcao_atualiza() {
  // medição do tempo de cada volta do loop (aparece no comando "tempo")
  uint32_t us = micros();
  P.saude.ciclo_us = us - tCicloAnt;
  if (P.saude.ciclo_us > P.saude.ciclo_us_max) P.saude.ciclo_us_max = P.saude.ciclo_us;
  tCicloAnt = us;
  P.t_ms = millis();

  chave_atualiza();          // chave DIP com filtro de 30 ms (chave.cpp)
  tof_atualiza();            // sem a tarefa do núcleo 0: leitura dos ToF e passo do rastreador aqui
  oponente_funde(P.op);      // estimativa pronta do oponente (oponente.cpp + rastreador.cpp)
  // Contato: oponente visto colado na frente. Depois que ele cola, o ToF pode perder a leitura
  // (alvo encostado na lente), então o contato continua valendo por T_SEGURA_CONTATO_MS.
  if (P.op.visto && P.op.dist_mm >= 0 && P.op.dist_mm <= CONTATO_MM && fabsf(P.op.angulo) <= 30) {
    tContato = P.t_ms; teveContato = true;
  }
  P.op.contato = teveContato && P.t_ms - tContato <= T_SEGURA_CONTATO_MS;
  atualizaRumo();
  borda_atualiza(P.borda);   // leitura e confirmação da linha branca (borda.cpp)
  oponente_borda_tras(P.borda.tras);
  // Localização no dojô com o movimento estimado das rodas e a borda (arena.cpp). O modelo das
  // rodas não sabe se o robô está preso: com o oponente encostado em qualquer lado (visto pelo
  // rastreador ou colado num ToF, abaixo da leitura mínima), ou com a borda vista por mais de
  // 150 ms seguidos (fuga que não sai do lugar, robô travado entre a borda e o oponente), o
  // movimento entra como incerto.
  if (!P.borda.detectada) tBordaSem = P.t_ms;
  bool perto = P.op.existe >= 0.5f && P.rumo.dist_mm > 0 && P.rumo.dist_mm < 240;
  P.op.colados = tof_colados(150);
  bool incerto = P.op.contato || perto || P.op.colados || P.t_ms - tBordaSem > 150;
  arena_atualiza(cinematica_v_mms(), cinematica_w_dps(), P.borda.cru, P.borda.mascara, incerto);
  arena_le(P.arena);
  atualizaSaude();

  // Botão de start: nível baixo significa apertado. O debounce ignora mudanças com menos
  // de 30 ms e só a descida (apertar) gera o evento P.start.
  bool apertado = digitalRead(PINO_START) == LOW;
  P.start = false;
  if (apertado != startAnt && P.t_ms - tStartMud > 30) {
    tStartMud = P.t_ms;
    startAnt = apertado;
    if (apertado) P.start = true;
  }
}

// Nova tentativa automática nos sensores que caíram, liberada só com o robô parado, porque
// religar um sensor bloqueia o I2C por uns 20 ms. Quem faz a tentativa é a tarefa dos sensores.
void percepcao_manutencao(bool parado) {
  static bool ant = false;
  tof_libera_manutencao(parado);
  if (parado && !ant) tCicloAnt = micros();
  ant = parado;
}
