// ponte_robo.cpp
// Tabela FwApi de um robô. Compilado duas vezes: com FW_NS=fa e FW_API=fw_api_a, e com FW_NS=fb
// e FW_API=fw_api_b. O loop repete o loop() do sumo_esp32.ino, sem o debug pela USB.
#include "fw_pre.h"
namespace FW_NS {
#include "config.h"
#include "percepcao.h"
#include "estados.h"
#include "cinematica.h"
#include "motores.h"
#include "chave.h"
#include "oponente.h"
#include "rastreador.h"
#include "antitrava.h"
}
#include "ponte.h"

namespace FW_NS {
static void init() {
  motores_init();
  cinematica_init();
  percepcao_init();
  estados_init();
}
static void loop() {
  percepcao_atualiza();
  estados_passo();
  percepcao_manutencao(estado_atual() == ESPERA && !estados_teste_ativo());
}
static void le(FwLeitura& l) {
  l.estado = (int)estado_atual();
  l.opVisto = P.op.visto;
  l.opX = P.op.x; l.opY = P.op.y; l.opAng = P.op.angulo;
  l.arenaValida = P.arena.valida; l.arenaConfiavel = P.arena.confiavel;
  l.arenaCx = P.arena.cx; l.arenaCy = P.arena.cy; l.arenaSigma = P.arena.sigma;
  l.estrategia = (int)estados_estrategia();
  l.escuro = estados_escuro();
  l.travado = estados_travado();
  l.destravas = estados_destravas();
  l.aquecimento = cinematica_aquecimento_k();
  l.limiteTermico = cinematica_limite_termico();
  l.bordaMascara = P.borda.mascara;
  l.bordaCru = P.borda.cru;
  l.tofMascara = P.op.mascara;
  for (uint8_t i = 0; i < 8; i++) l.tof[i] = i < N_TOF ? tof_mm(i) : -1;
  l.opExiste = P.op.existe;
  l.opConfianca = P.op.confianca;
#if ANTI_TRAVA
  l.antiTrava = antitrava_disparos();
#else
  l.antiTrava = 0;
#endif
}
static void manobra(int m) {
#if ANTI_TRAVA
  if (m >= 0) antitrava_define((uint8_t)m);
#else
  (void)m;
#endif
}
static const char* nomeEstado(int e) { return estado_nome((Estado)e); }
static int nEstados() { return N_ESTADOS; }
static const char* nomeEstrategia(int e) { return estrategia_nome((Estrategia)e); }
}  // namespace FW_NS

const FwApi FW_API = {FW_NS::init, FW_NS::loop, FW_NS::le, FW_NS::nomeEstado, FW_NS::nEstados, FW_NS::nomeEstrategia,
                      FW_NS::rastreador_mapa, MAPA_NB, MAPA_NR, MAPA_R0, MAPA_DR, FW_NS::manobra};
