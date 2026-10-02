// percepcao.cpp
// Este módulo junta todos os sensores em uma foto só, a struct P declarada no percepcao.h.
// A cada volta do loop ele chama o oponente.cpp (ToF e fusão), calcula o rumo, chama o
// borda.cpp (linha branca) e atualiza a saúde dos sensores. Um sensor novo no futuro
// (IMU, encoder) entra criando o módulo dele e preenchendo um campo novo de P aqui.

#include "percepcao.h"
#include "config.h"
#include "telemetria.h"
#include "oponente.h"
#include "borda.h"
#include <Wire.h>

Percepcao P;   // foto do mundo, lida pela máquina de estados (estados.cpp) e pelo debug.cpp

static bool     startAnt = false;   // último estado do botão de start
static uint32_t tStartMud = 0;      // momento da última mudança do botão (debounce)
static uint32_t tCicloAnt = 0;      // início da volta anterior do loop, para medir o ciclo
static uint32_t tRetry = 0;         // última tentativa de religar sensores falhos

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

  tof_atualiza();            // leitura dos ToF que já têm medida nova (oponente.cpp)
  oponente_funde(P.op);      // fusão das leituras em um oponente só (oponente.cpp + rastreador.cpp)
  atualizaRumo();
  borda_atualiza(P.borda);   // leitura e confirmação da linha branca (borda.cpp)
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

// O loop só chama esta função no estado ESPERA, porque religar um sensor bloqueia por alguns ms.
void percepcao_manutencao() {
  if (P.saude.tof_ok == N_TOF) return;
  if (millis() - tRetry < TOF_RETRY_MS) return;
  tRetry = millis();
  tof_reinicia_falhos();
  atualizaSaude();
  tCicloAnt = micros();                // esse ciclo longo não entra como pior ciclo
}
