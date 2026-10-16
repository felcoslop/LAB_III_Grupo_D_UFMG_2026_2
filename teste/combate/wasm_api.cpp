// wasm_api.cpp
// Funções exportadas para a página 3D (doc/site/combate.html). A página cria o módulo, chama
// cb_inicia com as escolhas da tela e, a cada quadro, cb_passo e cb_quadro, que devolve um vetor
// de floats na memória do módulo com o estado dos dois robôs (layout em QUADRO_*).
#include <cstdio>
#include <cstring>
#include <string>
#include "combate.h"

#define EXPORTA(nome) extern "C" __attribute__((export_name(#nome)))

using namespace cb;

static const int QUADRO_CAB = 8, QUADRO_ROBO = 56;
static float quadro[QUADRO_CAB + 2 * QUADRO_ROBO];
static float pessoas[1 + 3 * 8];
static std::string json;

EXPORTA(cb_inicia) void cb_inicia(uint32_t semente, int estrA, int estrB, int brancoA, int brancoB, int b1000,
                                  int posA, int posB, double tRound, double muEst, double muDin, int comPessoas) {
  Config c;
  c.semente = semente; c.estrA = estrA; c.estrB = estrB;
  c.brancoA = brancoA != 0; c.brancoB = brancoB != 0; c.b1000 = b1000 != 0;
  c.posA = posA; c.posB = posB; c.tRound = tRound; c.muEst = muEst; c.muDin = muDin; c.pessoas = comPessoas != 0;
  inicia(c);
}

EXPORTA(cb_passo) int cb_passo(double ms) { return passo(ms) ? 1 : 0; }

EXPORTA(cb_quadro) float* cb_quadro() {
  const Mundo& M = mundoAtual();
  const Resultado& R = resultado();
  float* q = quadro;
  q[0] = (float)M.t; q[1] = (float)tLiberado(); q[2] = M.contato ? 1 : 0; q[3] = (float)M.fContato;
  q[4] = R.fim ? 1 : 0; q[5] = (float)R.vencedor; q[6] = (float)R.motivo; q[7] = (float)R.tempo;
  for (int k = 0; k < 2; k++) {
    const Robo& rb = M.r[k];
    float* o = quadro + QUADRO_CAB + k * QUADRO_ROBO;
    double ex, ey;
    rb.eixo(ex, ey);
    FwLeitura l;
    leFirmware(k, l);
    float v[QUADRO_ROBO] = {
      (float)ex, (float)ey, (float)rb.th, (float)rb.angE, (float)rb.angD, (float)rb.arfagem, (float)rb.rolagem,
      rb.fora ? 1.f : 0.f, (float)rb.motivoFora, (float)rb.tFora,
      (float)l.estado, (float)rb.dutyE, (float)rb.dutyD, (float)rb.vBat, (float)rb.nE, (float)rb.nD, (float)rb.nS,
      (float)rb.iE, (float)rb.iD,
      l.opVisto ? 1.f : 0.f, l.opX, l.opY, l.arenaCx, l.arenaCy, l.arenaSigma, l.arenaConfiavel ? 1.f : 0.f,
      l.aquecimento, (float)l.travado, (float)rb.vx, (float)rb.vy, (float)rb.w, (float)l.estrategia,
      l.escuro ? 1.f : 0.f, (float)l.bordaMascara, (float)l.tofMascara, (float)rb.lift.v,
      (float)rb.p.motor.rpmLivre, (float)rb.p.refl, (float)l.destravas, l.arenaValida ? 1.f : 0.f,
      (float)l.tof[0], (float)l.tof[1], (float)l.tof[2], (float)l.tof[3], (float)l.tof[4], (float)l.tof[5],
      (float)l.tof[6], (float)l.tof[7], (float)l.bordaCru, l.opExiste, l.opConfianca, manualAtivo(k) ? 1.f : 0.f,
      0, 0, 0, 0};
    memcpy(o, v, sizeof(v));
  }
  return quadro;
}

EXPORTA(cb_pessoas) float* cb_pessoas() {
  const Mundo& M = mundoAtual();
  int n = (int)std::min<size_t>(8, M.pessoas.size());
  pessoas[0] = (float)n;
  for (int k = 0; k < n; k++) {
    pessoas[1 + 3 * k] = (float)M.pessoas[k].x;
    pessoas[2 + 3 * k] = (float)M.pessoas[k].y;
    pessoas[3 + 3 * k] = (float)M.pessoas[k].raio;
  }
  return pessoas;
}

EXPORTA(cb_n_estados) int cb_n_estados() { return api(0).nEstados(); }

// controle manual: PWM de -1 a 1 em cada roda, depois dos 5 s parados daquele robô
EXPORTA(cb_manual) void cb_manual(int robo, int ativo, double dutyE, double dutyD) { manual(robo, ativo != 0, dutyE, dutyD); }

// mapa do rastreador do robô: "nb,nr,r0,dr,células" no formato da linha M do painel
EXPORTA(cb_mapa) const char* cb_mapa(int robo) {
  static char buf[64 + 1024];
  const FwApi& a = api(robo);
  int n = snprintf(buf, 64, "%d,%d,%d,%d,", a.mapaNb, a.mapaNr, a.mapaR0, a.mapaDr);
  uint16_t k = a.mapa(buf + n, (uint16_t)(sizeof(buf) - n - 1));
  buf[n + k] = 0;
  return k ? buf : "";
}

// textos: 0 nome do estado i, 1 nome da estratégia i, 2 motivo do fim i
EXPORTA(cb_texto) const char* cb_texto(int qual, int i) {
  if (qual == 0) return api(0).nomeEstado(i);
  if (qual == 1) return api(0).nomeEstrategia(i);
  return nomeMotivo(i);
}

// eventos novos do Log dos dois firmwares, como JSON: [[t, robô, "texto"], ...]
EXPORTA(cb_eventos) const char* cb_eventos() {
  const Evento* e;
  int n = eventos(&e);
  json = "[";
  for (int k = 0; k < n; k++) {
    char b[160];
    std::string t;
    for (const char* c = e[k].texto; *c; c++) { if (*c == '"' || *c == '\\') t += '\\'; if ((unsigned char)*c >= 32) t += *c; }
    snprintf(b, sizeof(b), "%s[%.3f,%d,\"", k ? "," : "", e[k].t, e[k].robo);
    json += b; json += t; json += "\"]";
  }
  json += "]";
  return json.c_str();
}
