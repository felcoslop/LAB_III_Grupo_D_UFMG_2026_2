// mundo.cpp
// Física dos dois robôs, borda e ToF do simulador de luta (explicação no mundo.h).
#include "mundo.h"
#include <cmath>
#include <algorithm>

namespace sim {

static inline double sq(double v) { return v * v; }

// Robô montado com as medidas do STL (o mesmo do simulador de combate, teste/combate): rodas
// atrás, rampa de 117,7 mm à frente do eixo, 980 g com o centro de massa 26,9 mm à frente do
// eixo (77% do peso nos pneus) e a ponta da rampa de PLA arrastando no dojô.
Param paramNosso() {
  Param p;
  p.nome = "nosso";
  p.massa = 0.98;
  p.frente = 0.118; p.tras = 0.031; p.meioY = 0.0745; p.xCg = 0.0269;
  p.bitola = 0.1273; p.raio = 0.016;
  p.fracRodas = 0.77; p.muApoio = 0.35;
  p.lamina = 0.6;                 // rampa impressa de 17,7 graus, sem chapa de aço
  p.refl = 0.05;                  // corpo preto fosco
  return p;
}

// Tipos de oponente usados na bateria de lutas. Os números de motor são de robôs típicos da
// categoria: o rápido usa motor de alta rotação e pouco torque, o lento usa redução alta.
Param paramOponente(const std::string& tipo) {
  Param p;
  p.nome = tipo;
  if (tipo == "parado") {                 // caixa parada, motores desligados e redução travada
    p.freado = true; p.lamina = 0.4;
  } else if (tipo == "pesado") {          // parado, com pneu mais grudento e lâmina boa
    p.freado = true; p.mu = 1.25; p.lamina = 0.7; p.fracRodas = 0.85;
  } else if (tipo == "rapido") {          // 1,3 m/s, pouco torque
    p.raio = 0.02; p.rpmLivre = 650; p.torqueTravado = 0.045; p.iTravado = 2.0; p.iLivre = 0.15;
    p.vBat = 7.4; p.vNom = 7.4; p.lamina = 0.45; p.mu = 0.9;
  } else if (tipo == "lento") {           // 0,2 m/s, torque alto, pneu grudento e lâmina boa
    p.raio = 0.02; p.rpmLivre = 100; p.torqueTravado = 0.55; p.iTravado = 1.5; p.mu = 1.2;
    p.lamina = 0.7; p.fracRodas = 0.85;
  } else if (tipo == "errante") {         // anda pelo dojô e ataca de perto
    p.raio = 0.018; p.rpmLivre = 400; p.torqueTravado = 0.12; p.lamina = 0.5;
  } else {                                // espelho: mesmo motor e roda do nosso robô
    p = paramNosso(); p.nome = "espelho";
  }
  return p;
}

void Robo::paraMundo(double bx, double by, double& wx, double& wy) const {
  double c = std::cos(th), s = std::sin(th);
  wx = x + c * bx - s * by;
  wy = y + s * bx + c * by;
}
void Robo::paraCorpo(double wx, double wy, double& bx, double& by) const {
  double c = std::cos(th), s = std::sin(th), dx = wx - x, dy = wy - y;
  bx = c * dx + s * dy;
  by = -s * dx + c * dy;
}
void Robo::cantos(double cx[4], double cy[4]) const {
  const double sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, -1, 1};
  for (int k = 0; k < 4; k++) paraMundo(sx[k] > 0 ? p.frente : -p.tras, sy[k] * p.meioY, cx[k], cy[k]);
}

Mundo::Mundo(uint32_t semente) : rng(semente) {
  nosso.p = paramNosso();
  op.p = paramOponente("parado");
}

// Forças de tração, atrito lateral e apoio de um robô, mais a dinâmica de cada roda.
void Mundo::forcasRobo(Robo& r, double& fx, double& fy, double& tq, double dt) {
  const Param& p = r.p;
  fx = fy = tq = 0;
  if (r.fora) return;
  const double ux = std::cos(r.th), uy = std::sin(r.th);       // frente
  const double nx = -uy, ny = ux;                              // esquerda
  const double Rm = p.vNom / p.iTravado;
  const double wLivre = p.rpmLivre * 2 * M_PI / 60;
  const double kE = (p.vNom - p.iLivre * Rm) / wLivre;
  const double kT = p.torqueTravado / p.iTravado;
  const double tFric = kT * p.iLivre;
  double carga = p.massa * G * p.fracRodas + r.cargaExtra;
  if (carga < 0) carga = 0;
  const double N = carga / 2;
  const double s0 = 0.03;
  for (int k = 0; k < 2; k++) {
    Roda& rd = k == 0 ? r.rodaE : r.rodaD;
    double by = (k == 0 ? 1 : -1) * p.bitola / 2;
    double rx = -uy * by, ry = ux * by;                        // braço da roda no mundo
    double vpx = r.vx - r.w * ry, vpy = r.vy + r.w * rx;       // velocidade do ponto de apoio
    double vLong = vpx * ux + vpy * uy, vLat = vpx * nx + vpy * ny;
    double fLong, fLat = -p.mu * N * std::tanh(vLat / s0);
    if (p.freado) {
      rd.w = 0;
      fLong = -p.mu * N * std::tanh(vLong / s0);              // roda travada: só atrito
    } else {
      double slip = rd.w * p.raio - vLong;
      fLong = p.mu * N * std::tanh(slip / s0);
      double i = (rd.v - kE * rd.w) / Rm;
      double torque = kT * i - tFric * std::tanh(rd.w / 1.0);
      rd.w += (torque - fLong * p.raio) / p.jRoda * dt;
    }
    double Fx = fLong * ux + fLat * nx, Fy = fLong * uy + fLat * ny;
    fx += Fx; fy += Fy;
    tq += rx * Fy - ry * Fx;
  }
  // apoio (lâmina e rodízio) arrastando no chão, na frente do eixo
  double ax = p.frente * 0.95;
  double rx = ux * ax, ry = uy * ax;
  double vpx = r.vx - r.w * ry, vpy = r.vy + r.w * rx;
  double vm = std::sqrt(vpx * vpx + vpy * vpy);
  if (vm > 1e-6) {
    double Ns = p.massa * G * (1 - p.fracRodas);
    double f = p.muApoio * Ns * std::tanh(vm / s0);
    double Fx = -f * vpx / vm, Fy = -f * vpy / vm;
    fx += Fx; fy += Fy; tq += rx * Fy - ry * Fx;
  }
}

// Contato entre os dois robôs: caixas orientadas, eixo de menor penetração.
void Mundo::contatoRobos(double& fxA, double& fyA, double& tqA, double& fxB, double& fyB, double& tqB) {
  fxA = fyA = tqA = fxB = fyB = tqB = 0;
  nosso.cargaExtra = op.cargaExtra = 0;
  contato = false;
  if (nosso.fora || op.fora) return;
  const Robo& A = nosso; const Robo& B = op;
  double ax[2] = {std::cos(A.th), -std::sin(A.th)}, ay[2] = {std::sin(A.th), std::cos(A.th)};
  double bx[2] = {std::cos(B.th), -std::sin(B.th)}, by[2] = {std::sin(B.th), std::cos(B.th)};
  double eixos[4][2] = {{ax[0], ay[0]}, {ax[1], ay[1]}, {bx[0], by[0]}, {bx[1], by[1]}};
  // centro de cada caixa (o eixo das rodas nem sempre fica no meio do corpo)
  double cAx, cAy, cBx, cBy;
  A.paraMundo(0.5 * (A.p.frente - A.p.tras), 0, cAx, cAy);
  B.paraMundo(0.5 * (B.p.frente - B.p.tras), 0, cBx, cBy);
  const double hA = 0.5 * (A.p.frente + A.p.tras), hB = 0.5 * (B.p.frente + B.p.tras);
  double dx = cBx - cAx, dy = cBy - cAy;
  double melhor = 1e9, nX = 0, nY = 0;
  for (auto& e : eixos) {
    double ex = e[0], ey = e[1];
    double d = dx * ex + dy * ey;
    double eA = std::fabs(hA * (ax[0] * ex + ay[0] * ey)) + std::fabs(A.p.meioY * (ax[1] * ex + ay[1] * ey));
    double eB = std::fabs(hB * (bx[0] * ex + by[0] * ey)) + std::fabs(B.p.meioY * (bx[1] * ex + by[1] * ey));
    double ov = eA + eB - std::fabs(d);
    if (ov <= 0) return;                        // eixo separador: sem contato
    if (ov < melhor) { melhor = ov; double sg = d >= 0 ? 1 : -1; nX = ex * sg; nY = ey * sg; }
  }
  contato = true;
  // ponto de contato: média do canto de B mais fundo em A e do canto de A mais fundo em B
  double cxA[4], cyA[4], cxB[4], cyB[4];
  A.cantos(cxA, cyA); B.cantos(cxB, cyB);
  int iA = 0, iB = 0;
  for (int k = 1; k < 4; k++) {
    if (cxA[k] * nX + cyA[k] * nY > cxA[iA] * nX + cyA[iA] * nY) iA = k;
    if (cxB[k] * nX + cyB[k] * nY < cxB[iB] * nX + cyB[iB] * nY) iB = k;
  }
  double px = 0.5 * (cxA[iA] + cxB[iB]), py = 0.5 * (cyA[iA] + cyB[iB]);
  // velocidade relativa no ponto
  double rAx = px - A.x, rAy = py - A.y, rBx = px - B.x, rBy = py - B.y;
  double vAx = A.vx - A.w * rAy, vAy = A.vy + A.w * rAx;
  double vBx = B.vx - B.w * rBy, vBy = B.vy + B.w * rBx;
  double vrx = vBx - vAx, vry = vBy - vAy;
  double vn = vrx * nX + vry * nY;
  const double k = 3e4, c = 60;
  double Fn = k * melhor - c * vn;
  if (Fn < 0) Fn = 0;
  double tX = -nY, tY = nX;
  double vt = vrx * tX + vry * tY;
  double Ft = -0.4 * Fn * std::tanh(vt / 0.03);
  double Fx = Fn * nX + Ft * tX, Fy = Fn * nY + Ft * tY;      // força em B
  fxB = Fx; fyB = Fy; tqB = rBx * Fy - rBy * Fx;
  fxA = -Fx; fyA = -Fy; tqA = -(rAx * Fy - rAy * Fx);
  // lâmina: quem encosta de frente com a lâmina mais baixa tira peso das rodas do outro
  double frenteA = std::cos(A.th) * nX + std::sin(A.th) * nY;      // A encosta com a frente
  double frenteB = -(std::cos(B.th) * nX + std::sin(B.th) * nY);   // B encosta com a frente
  double carga = std::min(Fn, 1.0 * G);
  if (frenteA > 0.8 && A.p.lamina > B.p.lamina) { double d = (A.p.lamina - B.p.lamina) * carga * 0.6; op.cargaExtra -= d; nosso.cargaExtra += 0.5 * d; }
  if (frenteB > 0.8 && B.p.lamina > A.p.lamina) { double d = (B.p.lamina - A.p.lamina) * carga * 0.6; nosso.cargaExtra -= d; op.cargaExtra += 0.5 * d; }
}

void Mundo::integra(Robo& r, double fx, double fy, double tq, double dt) {
  if (r.fora) { r.vx = r.vy = r.w = 0; return; }
  double I = r.p.massa * (sq(r.p.frente + r.p.tras) + sq(2 * r.p.meioY)) / 12;
  r.vx += fx / r.p.massa * dt;
  r.vy += fy / r.p.massa * dt;
  r.w  += tq / I * dt;
  r.x += r.vx * dt; r.y += r.vy * dt; r.th += r.w * dt;
  if (r.th > M_PI) r.th -= 2 * M_PI;
  if (r.th < -M_PI) r.th += 2 * M_PI;
}

// Regra: o robô sai quando o centro de massa passa da borda (tomba para fora) ou quando uma roda perde
// o apoio do dojô (o canto do corpo encosta no chão de fora).
void Mundo::confereFora(Robo& r) {
  if (r.fora) return;
  double gx, gy;
  r.paraMundo(r.p.xCg, 0, gx, gy);
  bool sai = std::hypot(gx, gy) > R_DOJO;
  for (int k = 0; k < 2 && !sai; k++) {
    double wx, wy;
    r.paraMundo(0, (k ? -1 : 1) * r.p.bitola / 2, wx, wy);
    if (std::hypot(wx, wy) > R_DOJO) sai = true;
  }
  if (sai) { r.fora = true; r.tFora = t; }
}

void Mundo::passo(double dt) {
  double fA[3], fB[3], cA[3], cB[3];
  contatoRobos(cA[0], cA[1], cA[2], cB[0], cB[1], cB[2]);
  if (contato) { tUltContato = t; tContatoTotal += dt; }
  forcasRobo(nosso, fA[0], fA[1], fA[2], dt);
  forcasRobo(op, fB[0], fB[1], fB[2], dt);
  integra(nosso, fA[0] + cA[0], fA[1] + cA[1], fA[2] + cA[2], dt);
  integra(op, fB[0] + cB[0], fB[1] + cB[1], fB[2] + cB[2], dt);
  t += dt;
  confereFora(nosso);
  confereFora(op);
}

// TCRT5000 a 3 a 8 mm do chão: preto perto de 1300 mV, branco perto de 260 mV e, fora do dojô
// (chão 2,5 cm mais baixo), quase nada de luz volta e a leitura fica alta.
double Mundo::bordaMv(double bxmm, double bymm) {
  double wx, wy;
  nosso.paraMundo(bxmm / 1000, bymm / 1000, wx, wy);
  double r = std::hypot(wx, wy);
  double mv;
  if (r > R_DOJO)              mv = 2400 + normal(80);
  else if (r > R_DOJO - FAIXA) mv = 260 + normal(30);
  else                         mv = 1300 + normal(60);
  if (uniforme(0, 1) < arranhoes) mv = uniforme(0, 1) < 0.5 ? 300 : 700;   // arranhão ou poeira por um instante
  return std::max(0.0, std::min(3300.0, mv));
}

// Raio contra um segmento: distância t > 0 até a interseção (ou -1) e o cosseno de incidência.
static double raioSegmento(double ox, double oy, double dx, double dy, double ax, double ay, double bx, double by, double& cosInc) {
  double ex = bx - ax, ey = by - ay;
  double den = dx * ey - dy * ex;
  if (std::fabs(den) < 1e-12) return -1;
  double t = ((ax - ox) * ey - (ay - oy) * ex) / den;
  double u = ((ax - ox) * dy - (ay - oy) * dx) / den;
  if (t <= 0 || u < 0 || u > 1) return -1;
  double el = std::hypot(ex, ey);
  cosInc = std::fabs((dx * ey - dy * ex) / el);      // normal do segmento contra a direção do raio
  return t;
}

// Distância geométrica até o primeiro objeto no eixo do sensor (para medir cobertura nos testes)
double Mundo::alcanceVisivel(double sx, double sy, double angGraus, int* quem) {
  double wx, wy;
  nosso.paraMundo(sx / 1000, sy / 1000, wx, wy);
  double a = nosso.th + angGraus * M_PI / 180;
  double dx = std::cos(a), dy = std::sin(a), melhor = 1e9;
  if (quem) *quem = -1;
  if (!op.fora) {
    double cx[4], cy[4], ci;
    op.cantos(cx, cy);
    for (int k = 0; k < 4; k++) { double t = raioSegmento(wx, wy, dx, dy, cx[k], cy[k], cx[(k + 1) % 4], cy[(k + 1) % 4], ci); if (t > 0 && t < melhor) { melhor = t; if (quem) *quem = 0; } }
  }
  return melhor < 1e8 ? melhor * 1000 : -1;
}

// ToF: 9 raios no cone de 25 graus. Cada objeto atingido soma sinal (refletância, cobertura,
// incidência e 1/d²); o objeto de sinal mais forte define a leitura, e sinal abaixo do limiar
// do perfil (com flutuação) vira "nada" (8190), como o VL53L0X faz com alvo escuro e longe.
int Mundo::tofMede(double sx, double sy, double angGraus, const PerfilToF& pf, int* quem) {
  if (quem) *quem = -1;
  double wx, wy;
  nosso.paraMundo(sx / 1000, sy / 1000, wx, wy);
  const int NR = 9;
  double sinalObj[16] = {0}, distObj[16] = {0}, pesoObj[16] = {0};
  int nObj = 1 + (int)fora.size();
  if (nObj > 16) nObj = 16;
  double pesoTotal = 0;
  for (int k = 0; k < NR; k++) {
    double frac = (k - (NR - 1) / 2.0) / ((NR - 1) / 2.0);       // -1 a 1 no cone
    double wRaio = std::exp(-1.5 * frac * frac);                  // feixe mais forte no centro
    pesoTotal += wRaio;
    double a = nosso.th + (angGraus + 12.5 * frac) * M_PI / 180;
    double dx = std::cos(a), dy = std::sin(a);
    double tMin = 1e9, cMin = 1; int oMin = -1;
    if (!op.fora) {
      double cx[4], cy[4];
      op.cantos(cx, cy);
      for (int e = 0; e < 4; e++) {
        double ci;
        double t = raioSegmento(wx, wy, dx, dy, cx[e], cy[e], cx[(e + 1) % 4], cy[(e + 1) % 4], ci);
        if (t > 0 && t < tMin) { tMin = t; cMin = ci; oMin = 0; }
      }
    }
    for (size_t j = 0; j < fora.size() && (int)j + 1 < 16; j++) {
      const Objeto& o = fora[j];
      double fx = wx - o.x, fy = wy - o.y;
      double b = fx * dx + fy * dy, cc = fx * fx + fy * fy - o.raio * o.raio;
      double disc = b * b - cc;
      if (disc < 0) continue;
      double t = -b - std::sqrt(disc);
      if (t > 0 && t < tMin) { tMin = t; cMin = 1; oMin = (int)j + 1; }
    }
    if (oMin < 0) continue;
    double refl = oMin == 0 ? op.p.refl : fora[oMin - 1].refl;
    double inc = cMin;
    if (oMin == 0 && op.p.furtivo) inc = std::pow(cMin, 4);
    double d10 = tMin / 0.1;
    sinalObj[oMin] += refl * inc * wRaio / (d10 * d10);
    distObj[oMin] += tMin * wRaio;
    pesoObj[oMin] += wRaio;
  }
  int melhor = -1; double sMax = 0;
  for (int o = 0; o < nObj; o++) if (sinalObj[o] > sMax) { sMax = sinalObj[o]; melhor = o; }
  if (uniforme(0, 1) < pf.falsa) return (int)uniforme(30, 900);   // leitura falsa (luz ambiente, reflexo)
  if (melhor < 0) return 8190;
  double sinal = sMax / pesoTotal * std::exp(normal(0.2));
  if (sinal < pf.limiar) return 8190;
  double d = distObj[melhor] / pesoObj[melhor] * 1000;
  if (d > pf.alcanceMax) return 8190;
  double sg = pf.ruido * (4 + 0.015 * d) * (1 + pf.limiar / sinal);
  int mm = (int)std::lround(d + normal(sg));
  if (quem) *quem = melhor;
  return mm < 1 ? 1 : mm;
}

}  // namespace sim
