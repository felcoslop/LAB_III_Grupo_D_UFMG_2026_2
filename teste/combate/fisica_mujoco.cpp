// fisica_mujoco.cpp
// Segunda física do simulador de combate: o MuJoCo no lugar das contas à mão do fisica.cpp.
// Explicação, parâmetros e limites no README ("Física do MuJoCo") e no fisica_mujoco.h.
//
// O que o MuJoCo resolve: o contato em 3D entre as malhas do chassi dos dois robôs (a rampa de um
// entrando por baixo da lâmina do outro, o levantamento e o tombamento), a ponta da rampa
// arrastando no dojô, o pneu em cada roda e a inércia de cada corpo. O que continua do fisica.cpp:
// o modelo do motor DC (força contraeletromotriz, queda da bateria e da ponte H), a regra de quem
// saiu do dojô e os sensores (ToF e TCRT5000), que leem a posição e o rumo que este módulo grava
// no Robo a cada passo.
#include "fisica.h"

#ifdef CB_MUJOCO
#include <mujoco/mujoco.h>
#include <cmath>
#include <cstdio>
#include <memory>
#include <cstdlib>
#include <cstring>
#include <string>

#ifndef CB_MALHAS
#define CB_MALHAS "teste/combate/mujoco/malhas"
#endif

namespace cb {

namespace {

const double DT_MJ = 0.0005;       // passo do MuJoCo (s); o firmware roda a cada 1 ms
const double Z_QUEDA = -0.02;      // o corpo desceu 2 cm abaixo do dojô: caiu para fora
const char* const PECAS[] = {"base", "rampa1", "rampa2", "topo", "latL", "latR"};

struct Estado {
  mjModel* m = nullptr;
  mjData* d = nullptr;
  double phi = 0;          // a cena do MuJoCo é girada de phi em relação à do Mundo (ver abaixo)
  double acc = 0;          // tempo do firmware ainda não passado ao MuJoCo
  int corpo[2] = {0, 0};   // índice do corpo livre de cada robô no modelo
  int qpos[2] = {0, 0};    // endereço do corpo livre em qpos
  int dofRoda[2][2] = {{0, 0}, {0, 0}};   // [robô][0 esquerda, 1 direita]
  int geomRoda[2][2] = {{0, 0}, {0, 0}};
  int geomChao = 0;
  int robo[256];           // robô dono de cada geom (-1 chão)
  ~Estado() {
    if (d) mj_deleteData(d);
    if (m) mj_deleteModel(m);
  }
};

// O MuJoCo desempata contatos pela ordem e pela posição dos corpos, e no começo ele favorecia
// em 84 a 16 o robô que largava em +x contra um idêntico. Cada luta gira a cena inteira de um
// ângulo sorteado (o dojô é redondo) e sorteia qual robô entra primeiro no modelo; as posições
// voltam ao referencial do Mundo na saída. Com isso, dois robôs iguais empatam em 50 a 50.
void gira(double phi, double x, double y, double& ox, double& oy) {
  const double c = std::cos(phi), s = std::sin(phi);
  ox = c * x - s * y;
  oy = s * x + c * y;
}

std::string xmlRobo(const Robo& rb, int k, int ordem) {
  const Param& p = rb.p;
  const Motor& mt = p.motor;
  const double mRoda = 0.02;
  const double arm = std::max(0.0, mt.jRoda - 0.5 * mRoda * R_RODA * R_RODA);
  char b[4096];
  std::string malhas;
  // O STEP deixa a ponta da rampa 3 mm acima do dojô com o robô nivelado. CB_PONTA_MM (em mm) baixa
  // as malhas para testar outra altura de impressão, como o `ponta` da física à mão (0,1 a 0,6 mm).
  const char* pontaEnv = getenv("CB_PONTA_MM");
  const double dz = pontaEnv ? (atof(pontaEnv) - 3.0) * 1e-3 : 0.0;
  for (const char* n : PECAS) {
    snprintf(b, sizeof b, "<geom name=\"r%d_%s\" type=\"mesh\" mesh=\"%s\" pos=\"0 0 %.5f\" friction=\"%.3f 0.005 0.0001\" rgba=\"0.3 0.3 0.35 1\"/>",
             k, n, n, dz, p.muPP);
    malhas += b;
  }
  const double muPneu = 0.5 * (p.muEst + p.muDin);
  std::string rodas;
  for (int s = 0; s < 2; s++) {
    snprintf(b, sizeof b,
             "<body name=\"r%d_roda%d\" pos=\"0 %.5f %.5f\"><joint name=\"r%d_j%d\" type=\"hinge\" axis=\"0 1 0\" armature=\"%.6g\"/>"
             "<geom name=\"r%d_pneu%d\" type=\"cylinder\" size=\"%.5f 0.0101\" euler=\"90 0 0\" mass=\"%.4f\" friction=\"%.3f 0.005 0.0001\" rgba=\"0.9 0.8 0.2 1\"/></body>",
             k, s, (s == 0 ? 1 : -1) * BITOLA / 2, R_RODA, k, s, arm, k, s, R_RODA, mRoda, muPneu);
    rodas += b;
  }
  (void)ordem;
  snprintf(b, sizeof b,
           "<body name=\"r%d\" pos=\"0 0 0.0005\"><freejoint name=\"r%d_livre\"/>"
           "<inertial pos=\"%.5f 0 %.5f\" mass=\"%.4f\" diaginertia=\"1.5e-3 2.5e-3 %.6g\"/>",
           k, k, p.xCg, p.hCg, p.massa - 2 * mRoda, p.izz);
  return std::string(b) + malhas + rodas + "</body>";
}

mjModel* constroi(const Mundo& w, const int ordem[2]) {
  std::string xml;
  char b[1024];
  snprintf(b, sizeof b,
           "<mujoco><option timestep=\"%.6f\" gravity=\"0 0 %.5f\" integrator=\"implicitfast\"/>"
           "<default><geom solref=\"0.004 1\" solimp=\"0.95 0.99 0.001\"/></default><asset>",
           DT_MJ, -G);
  xml = b;
  for (const char* n : PECAS) {
    snprintf(b, sizeof b, "<mesh name=\"%s\" file=\"%s/%s.stl\" scale=\"0.001 0.001 0.001\"/>", n, CB_MALHAS, n);
    xml += b;
  }
  // dojô: cilindro de 77 cm com a face de cima em z = 0; o chão de fora fica sem nada (quem passa da
  // borda cai e a regra de saída o pega); atrito da ponta da rampa de PLA no dojô pintado
  snprintf(b, sizeof b,
           "</asset><worldbody><geom name=\"dojo\" type=\"cylinder\" size=\"%.4f 0.0125\" pos=\"0 0 -0.0125\" friction=\"%.3f 0.005 0.0001\" rgba=\"0.1 0.1 0.1 1\"/>",
           R_DOJO, w.r[0].p.muPla);
  xml += b;
  for (int q = 0; q < 2; q++) xml += xmlRobo(w.r[ordem[q]], ordem[q], q);
  xml += "</worldbody></mujoco>";
  char erro[1000] = "";
  mjSpec* spec = mj_parseXMLString(xml.c_str(), nullptr, erro, sizeof erro);
  if (!spec) { fprintf(stderr, "MuJoCo: %s\n", erro); exit(2); }
  mjModel* m = mj_compile(spec, nullptr);
  if (!m) { fprintf(stderr, "MuJoCo: %s\n", mjs_getError(spec)); exit(2); }
  mj_deleteSpec(spec);
  return m;
}

void iniciaEstado(Estado& E, Mundo& w) {
  int ordem[2] = {0, 1};
  E.phi = w.uniforme(0, 2 * M_PI);
  if (w.uniforme(0, 1) < 0.5) { ordem[0] = 1; ordem[1] = 0; }
  E.m = constroi(w, ordem);
  E.d = mj_makeData(E.m);
  for (int k = 0; k < 2; k++) {
    char nome[32];
    snprintf(nome, sizeof nome, "r%d", k);
    E.corpo[k] = mj_name2id(E.m, mjOBJ_BODY, nome);
    snprintf(nome, sizeof nome, "r%d_livre", k);
    E.qpos[k] = E.m->jnt_qposadr[mj_name2id(E.m, mjOBJ_JOINT, nome)];
    for (int s = 0; s < 2; s++) {
      snprintf(nome, sizeof nome, "r%d_j%d", k, s);
      E.dofRoda[k][s] = E.m->jnt_dofadr[mj_name2id(E.m, mjOBJ_JOINT, nome)];
      snprintf(nome, sizeof nome, "r%d_pneu%d", k, s);
      E.geomRoda[k][s] = mj_name2id(E.m, mjOBJ_GEOM, nome);
    }
    // pose inicial do Mundo (eixo no chão, rumo) girada de phi
    double ex, ey;
    w.r[k].eixo(ex, ey);
    double gx, gy;
    gira(E.phi, ex, ey, gx, gy);
    const double th = w.r[k].th + E.phi;
    double* q = E.d->qpos + E.qpos[k];
    q[0] = gx; q[1] = gy; q[2] = 0.0005;
    q[3] = std::cos(th / 2); q[4] = 0; q[5] = 0; q[6] = std::sin(th / 2);
  }
  E.geomChao = mj_name2id(E.m, mjOBJ_GEOM, "dojo");
  for (int g = 0; g < E.m->ngeom && g < 256; g++) {
    int b = E.m->geom_bodyid[g];
    int dono = -1;
    for (int k = 0; k < 2; k++) {
      // os corpos das rodas são filhos do corpo livre
      if (b == E.corpo[k] || E.m->body_parentid[b] == E.corpo[k]) dono = k;
    }
    E.robo[g] = dono;
  }
  mj_forward(E.m, E.d);
}

// torque de cada motor, com o modelo do fisica.cpp (forcasRobo)
void motores(Estado& E, Robo& rb, int k) {
  const Param& p = rb.p;
  const Motor& m = p.motor;
  const double Rm = m.vNom / m.iTravado;
  double iBat = rb.dutyE * rb.iE + rb.dutyD * rb.iD;
  rb.vBat = std::max(6.0, p.vAberta - p.rBat * iBat);
  const double vDisp = rb.vBat - p.queda;
  for (int s = 0; s < 2; s++) {
    const double wLivre = m.rpmLivre * (1 + p.varRpm[s]) * 2 * M_PI / 60;
    const double kE = (m.vNom - m.iLivre * Rm) / wLivre;
    const double kT = m.torqueTravado * (1 + p.varTorque[s]) / m.iTravado;
    const double tFric = kT * m.iLivre;
    const double wr = E.d->qvel[E.dofRoda[k][s]];
    const double duty = s == 0 ? rb.dutyE : rb.dutyD;
    const double i = (duty * vDisp - kE * wr) / Rm;
    (s == 0 ? rb.iE : rb.iD) = i;
    E.d->qfrc_applied[E.dofRoda[k][s]] = kT * i - tFric * std::tanh(wr / 1.0);
  }
}

// grava no Robo a pose, as velocidades e as cargas lidas do MuJoCo
void leMundo(Estado& E, Mundo& w, double dt, bool contatoAgora) {
  const mjData* d = E.d;
  for (int k = 0; k < 2; k++) {
    Robo& rb = w.r[k];
    const mjtNum* R = d->xmat + 9 * E.corpo[k];
    const double yaw = std::atan2(R[3], R[0]);
    double ex, ey;
    gira(-E.phi, d->xpos[3 * E.corpo[k]], d->xpos[3 * E.corpo[k] + 1], ex, ey);
    rb.th = yaw - E.phi;
    while (rb.th > M_PI) rb.th -= 2 * M_PI;
    while (rb.th < -M_PI) rb.th += 2 * M_PI;
    rb.x = ex + std::cos(rb.th) * rb.p.xCg;
    rb.y = ey + std::sin(rb.th) * rb.p.xCg;
    rb.arfagem = std::asin(std::max(-1.0, std::min(1.0, (double)R[6])));
    rb.rolagem = std::asin(std::max(-1.0, std::min(1.0, (double)R[7])));
    mjtNum v[6];
    mj_objectVelocity(E.m, d, mjOBJ_BODY, E.corpo[k], v, 0);
    gira(-E.phi, v[3], v[4], rb.vx, rb.vy);
    rb.w = v[2];
    rb.wE = d->qvel[E.dofRoda[k][0]];
    rb.wD = d->qvel[E.dofRoda[k][1]];
    rb.angE += rb.wE * dt;
    rb.angD += rb.wD * dt;
    rb.nE = rb.nD = rb.nS = 0;
    rb.lift = rb.prensa = Lift();
  }
  // forças de contato: pneus e ponta no dojô (cargas) e um robô no outro (levantamento)
  double fz[2] = {0, 0}, px[2] = {0, 0}, py[2] = {0, 0}, pz[2] = {0, 0}, soma[2] = {0, 0};
  double fCont = 0;
  for (int c = 0; c < d->ncon; c++) {
    const mjContact& ct = d->contact[c];
    mjtNum f6[6];
    mj_contactForce(E.m, d, c, f6);
    const int g1 = ct.geom1, g2 = ct.geom2;
    const int r1 = g1 < 256 ? E.robo[g1] : -1, r2 = g2 < 256 ? E.robo[g2] : -1;
    if (r1 >= 0 && r2 >= 0 && r1 != r2) {
      // força no geom2 = frame^T . f; vertical, e o ponto de aplicação no corpo de cada robô
      const double fv = ct.frame[0] * f6[0] + ct.frame[3] * f6[1] + ct.frame[6] * f6[2];   // x
      const double fw = ct.frame[1] * f6[0] + ct.frame[4] * f6[1] + ct.frame[7] * f6[2];   // y
      const double fzz = ct.frame[2] * f6[0] + ct.frame[5] * f6[1] + ct.frame[8] * f6[2];  // z
      (void)fv; (void)fw;
      fCont += f6[0];
      for (int q = 0; q < 2; q++) {
        const int rr = q == 0 ? r2 : r1;           // quem recebe a força
        const double sinal = q == 0 ? 1 : -1;      // geom2 recebe +, geom1 recebe -
        fz[rr] += sinal * fzz;
        if (sinal * fzz > 0) {
          px[rr] += sinal * fzz * ct.pos[0]; py[rr] += sinal * fzz * ct.pos[1]; pz[rr] += sinal * fzz * ct.pos[2];
          soma[rr] += sinal * fzz;
        }
      }
    } else if ((r1 >= 0) != (r2 >= 0)) {
      const int rr = r1 >= 0 ? r1 : r2, g = r1 >= 0 ? g1 : g2;
      Robo& rb = w.r[rr];
      if (g == E.geomRoda[rr][0]) rb.nE += f6[0];
      else if (g == E.geomRoda[rr][1]) rb.nD += f6[0];
      else rb.nS += f6[0];
    }
  }
  for (int k = 0; k < 2; k++) {
    Robo& rb = w.r[k];
    if (fz[k] > 0 && soma[k] > 0) {
      // ponto médio do contato (ponderado pela força) no referencial do corpo, em metros
      double gx, gy;
      gira(-E.phi, px[k] / soma[k], py[k] / soma[k], gx, gy);
      double bx, by;
      rb.paraCorpo(gx, gy, bx, by);
      rb.lift = {fz[k], bx, by};
    } else if (fz[k] < 0) {
      rb.prensa = {fz[k], 0, 0};
    }
  }
  w.contato = contatoAgora;
  w.fContato = fCont;
  (void)dt;
}

// quem saiu: roda, centro de massa ou corpo que desceu abaixo do dojô (a regra do fisica.cpp,
// confereFora, sem o apoio da quina, que o MuJoCo resolve sozinho)
void confereForaMj(Mundo& w, const Estado& E, Robo& rb, int k) {
  if (rb.fora) return;
  int motivo = 0;
  for (int s = 0; s < 2 && !motivo; s++) {
    double wx, wy;
    rb.paraMundo(0, (s ? -1 : 1) * BITOLA / 2, wx, wy);
    if (std::hypot(wx, wy) > R_DOJO) motivo = 1;
  }
  if (!motivo && std::hypot(rb.x, rb.y) > R_DOJO) motivo = 2;
  if (!motivo && E.d->xpos[3 * E.corpo[k] + 2] < Z_QUEDA) motivo = 3;
  if (motivo) { rb.fora = true; rb.tFora = w.t; rb.motivoFora = motivo; }
}

void passoMj(Estado& E, Mundo& w, double dt) {
  if (w.r[0].fora || w.r[1].fora) { w.contato = false; return; }
  E.acc += dt;
  bool contatoAgora = false;
  while (E.acc >= DT_MJ - 1e-12) {
    for (int k = 0; k < 2; k++) motores(E, w.r[k], k);
    mj_step(E.m, E.d);
    E.acc -= DT_MJ;
    for (int c = 0; c < E.d->ncon; c++) {
      const mjContact& ct = E.d->contact[c];
      if (ct.geom1 < 256 && ct.geom2 < 256 && E.robo[ct.geom1] >= 0 && E.robo[ct.geom2] >= 0 && E.robo[ct.geom1] != E.robo[ct.geom2]) contatoAgora = true;
    }
  }
  leMundo(E, w, dt, contatoAgora);
  if (w.contato) { w.tUltContato = w.t; w.tContatoTotal += dt; }
  for (int k = 0; k < 2; k++) confereForaMj(w, E, w.r[k], k);
}

}  // namespace

void mujocoLiga(Mundo& w) {
  auto est = std::make_shared<Estado>();
  bool pronto = false;
  w.motorFisico = [est, pronto](Mundo& m, double dt) mutable {
    // o modelo nasce no primeiro passo, depois que o combate posicionou e configurou os robôs
    if (!pronto) { iniciaEstado(*est, m); pronto = true; }
    passoMj(*est, m, dt);
  };
}

}  // namespace cb

#else   // sem o MuJoCo na compilação

#include <cstdio>
#include <cstdlib>
namespace cb {
void mujocoLiga(Mundo&) {
#ifdef __wasm__
  // No WebAssembly da página 3D a opção nunca é pedida; sem o exit(), o módulo não importa proc_exit do WASI
  // (a página só fornece fd_write e parentes), e o navegador deixa de carregá-lo.
#else
  fprintf(stderr, "este executável foi compilado sem o MuJoCo (MUJOCO=1 bash teste/combate/compila.sh)\n");
  exit(2);
#endif
}
}  // namespace cb
#endif
