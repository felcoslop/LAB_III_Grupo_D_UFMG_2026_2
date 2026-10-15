// combate.cpp
// Laço do simulador de combate: boot dos dois firmwares, posição inicial pela regra da RoboCore,
// botão de start de cada operador, loop de cada firmware a cada 1 ms, física a cada 0,125 ms e
// fim do round (explicação no combate.h).
#include "combate.h"
#include "hal_combate.h"
#include "Preferences.h"
#include <cmath>
#include <cstring>
#include <vector>

namespace cb {

static Mundo*      M = nullptr;
static Config      C;
static Resultado   R;
static Estatistica E;
static uint64_t    tBotao[2] = {0, 0};
static bool        apertou[2] = {false, false};
static double      tLib = -1;
static uint64_t    proxEstat = 0;
static std::vector<Evento> ev;
static size_t      evLidos = 0;
static bool        manualOn[2] = {false, false};
static double      manualDuty[2][2] = {{0, 0}, {0, 0}};

void manual(int k, bool ativo, double dE, double dD) {
  if (k < 0 || k > 1) return;
  manualOn[k] = ativo;
  manualDuty[k][0] = std::max(-1.0, std::min(1.0, dE));
  manualDuty[k][1] = std::max(-1.0, std::min(1.0, dD));
}
bool manualAtivo(int k) { return k >= 0 && k <= 1 && manualOn[k]; }
static const double DT_FIS = 0.000125;

const FwApi& api(int k) { return k == 0 ? fw_api_a : fw_api_b; }
const Mundo& mundoAtual() { return *M; }
const Config& configAtual() { return C; }
const Resultado& resultado() { return R; }
const Estatistica& estatistica() { return E; }
double agora() { return us * 1e-6; }
double tLiberado() { return tLib; }
void leFirmware(int k, FwLeitura& l) { int a = atual; atual = k; api(k).le(l); atual = a; }

const char* nomeMotivo(int m) {
  static const char* N[] = {"", "empurrado para fora", "saiu sozinho", "tempo esgotado", "os dois caíram"};
  return m >= 0 && m <= 4 ? N[m] : "";
}

static void guardaLinha(int robo, const char* linha) {
  if (linha[0] != '#') return;
  if (ev.size() > 4000) return;
  Evento e;
  e.t = us * 1e-6; e.robo = robo;
  strncpy(e.texto, linha, sizeof(e.texto) - 1);
  e.texto[sizeof(e.texto) - 1] = 0;
  ev.push_back(e);
}

int eventos(const Evento** lista) {
  int n = (int)(ev.size() - evLidos);
  *lista = n > 0 ? &ev[evLidos] : nullptr;
  evLidos = ev.size();
  return n;
}

// Cada robô fica inteiro na sua metade, atrás da linha Shikiri (linhas de 1 cm afastadas 10 cm
// pelos limites de fora, na classe de 1 kg), com folga para nenhum sensor de borda começar no
// branco. Rumo: de frente para o oponente, de lado (dois lados) ou de costas.
static void posiciona(Robo& rb, int lado, int jeito) {
  for (int tent = 0; tent < 2000; tent++) {
    double th;
    switch (jeito) {
      case 0: th = lado > 0 ? M_PI : 0; break;
      case 1: th = M_PI / 2; break;
      case 2: th = -M_PI / 2; break;
      default: th = lado > 0 ? 0 : M_PI;
    }
    th += M->normal(0.03);                    // o operador não alinha perfeito
    double ex = lado * M->uniforme(0.05, 0.30), ey = M->uniforme(-0.15, 0.15);
    rb.th = th;
    rb.x = ex + std::cos(th) * rb.p.xCg;
    rb.y = ey + std::sin(th) * rb.p.xCg;
    double cx[4], cy[4];
    cantos(rb, cx, cy);
    bool ok = true;
    for (int k = 0; k < 4; k++) {
      if (lado * cx[k] < 0.052) ok = false;
      if (std::hypot(cx[k], cy[k]) > R_DOJO - FAIXA - 0.012) ok = false;
    }
    if (ok) return;
  }
  rb.th = lado > 0 ? M_PI : 0;
  rb.x = lado * 0.15; rb.y = 0;
}

void inicia(const Config& c) {
  C = c;
  delete M;
  M = new Mundo(c.semente * 2654435761u + 17);
  mundo = M;
  halReinicia();
  Preferences::apagaTudo();
  aoLinha = guardaLinha;
  R = Resultado(); E = Estatistica();
  ev.clear(); evLidos = 0;
  tLib = -1; apertou[0] = apertou[1] = false;
  for (int k = 0; k < 2; k++) {
    Robo& rb = M->r[k];
    rb = Robo();
    rb.p.massa = c.massa;
    rb.p.muEst *= c.atrito; rb.p.muDin *= c.atrito;
    rb.p.restit = c.restit;
    rb.p.refl = (k == 0 ? c.brancoA : c.brancoB) ? 0.6 : 0.05;
    rb.p.motor = (k == 1 && c.b1000) ? motor1000() : motor319();
    rb.p.ponta = M->uniforme(0.1e-3, 0.6e-3);           // ponta da rampa impressa
    rb.p.vAberta = M->uniforme(12.0, 12.5);             // bateria carregada antes da luta
    rb.vBat = rb.p.vAberta;
    // motorredutores baratos variam alguns por cento entre si na rotação e no torque
    for (int q = 0; q < 2; q++) { rb.p.varRpm[q] = M->normal(0.03); rb.p.varTorque[q] = M->normal(0.05); }
  }
  int jA = c.posA >= 0 ? c.posA : (int)M->uniforme(0, 4);
  int jB = c.posB >= 0 ? c.posB : (int)M->uniforme(0, 4);
  posiciona(M->r[0], +1, jA);
  posiciona(M->r[1], -1, jB);
  E.jeito[0] = jA; E.jeito[1] = jB;
  for (int k = 0; k < 2; k++) M->ajustaApoioInicial(M->r[k]);
  M->pessoas.clear();
  if (c.pessoas) {
    // dois operadores e o juiz, fora da área do dojô depois do start
    double a0 = M->uniforme(-M_PI, M_PI);
    for (int k = 0; k < 3; k++) {
      double a = a0 + k * 2.1 + M->uniforme(-0.3, 0.3), rr = R_DOJO + M->uniforme(0.9, 1.3);
      M->pessoas.push_back({rr * std::cos(a), rr * std::sin(a), 0.11, M->uniforme(0.25, 0.5)});
    }
  }
  // chave DIP: estratégia (vias 1 a 3) e perfil escuro (via 4) quando o outro robô é preto
  int eB = c.estrB >= 0 ? c.estrB : (int)M->uniforme(0, 3);
  HR[0].dip = (uint8_t)((1 << c.estrA) | (c.brancoB ? 0 : 8));
  HR[1].dip = (uint8_t)((1 << eB) | (c.brancoA ? 0 : 8));
  // boot dos dois firmwares, com os robôs parados no dojô
  for (int k = 0; k < 2; k++) { atual = k; api(k).init(); }
  // sinal do juiz 1 s depois do boot; cada operador aperta com o próprio tempo de reação
  for (int k = 0; k < 2; k++) tBotao[k] = us + 1000000 + (uint64_t)M->uniforme(150000, 300000);
  M->t = us * 1e-6;
  proxEstat = us;
}

static void acaba(int vencedor, int motivo) {
  R.fim = true;
  R.vencedor = vencedor;
  R.motivo = motivo;
  R.tempo = tLib > 0 ? std::max(0.0, M->t - tLib) : 0;
  for (int k = 0; k < 2; k++) { FwLeitura l; leFirmware(k, l); E.destravas[k] = l.destravas; }
}

bool passo(double ms) {
  if (R.fim) return false;
  uint64_t alvo = us + (uint64_t)(ms * 1000);
  while (us < alvo && !R.fim) {
    // botões de start: cada um fica apertado por 50 ms
    for (int k = 0; k < 2; k++) {
      if (apertou[k]) continue;
      HR[k].botao = us >= tBotao[k];
      if (us >= tBotao[k] + 50000) {
        HR[k].botao = false; apertou[k] = true;
        double tl = tBotao[k] * 1e-6 + 5.0;
        if (tLib < 0 || tl < tLib) tLib = tl;
      }
    }
    // firmwares (loop de 1 ms)
    for (int k = 0; k < 2; k++) {
      atual = k;
      api(k).loop();
      M->r[k].dutyE = dutyMotor(k, true);
      M->r[k].dutyD = dutyMotor(k, false);
      if (manualOn[k]) {
        // o operador também respeita os 5 s parados depois do próprio start
        bool livre = apertou[k] && us >= tBotao[k] + 50000 + 5000000;
        M->r[k].dutyE = livre ? manualDuty[k][0] : 0;
        M->r[k].dutyD = livre ? manualDuty[k][1] : 0;
      }
    }
    // física até o próximo loop
    for (int q = 0; q < 8; q++) { M->passo(DT_FIS); us += 125; }
    M->t = us * 1e-6;
    // estatística
    if (M->contato) {
      E.tContato += 0.001;
      E.fMaxContato = std::max(E.fMaxContato, M->fContato);
      if (E.tPrimContato < 0 && tLib > 0) E.tPrimContato = M->t - tLib;
    }
    for (int k = 0; k < 2; k++) {
      const Robo& rb = M->r[k];
      double base = rb.p.massa * G * (1 - rb.p.xCg / X_PONTA);
      if (rb.lift.v > 0 && rb.nE + rb.nD < 0.7 * base) E.tLevantado[k] += 0.001;
    }
    if (us >= proxEstat) {
      proxEstat = us + 100000;
      for (int k = 0; k < 2; k++) { FwLeitura l; leFirmware(k, l); E.aquecMax[k] = std::max(E.aquecMax[k], (double)l.aquecimento); }
    }
    // fim do round
    bool fa = M->r[0].fora, fb = M->r[1].fora;
    if (fa || fb) {
      if (fa && fb) acaba(-1, 4);
      else {
        int perdedor = fa ? 0 : 1;
        bool empurrado = M->t - M->tUltContato < 0.3;
        acaba(1 - perdedor, empurrado ? 1 : 2);
      }
    } else if (tLib > 0 && M->t >= tLib + C.tRound) acaba(-1, 3);
  }
  return !R.fim;
}

}  // namespace cb
