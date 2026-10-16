// fisica.cpp
// Física dos dois robôs, sensores de borda e ToF do simulador de combate (explicação no fisica.h).
#include "fisica.h"
#include <cmath>
#include <algorithm>

namespace cb {

static inline double sq(double v) { return v * v; }
static const double KGFCM = 0.0980665;   // 1 kgf.cm em N.m

// Tabela do JGB37-520 de 12 V (RCDrone), linha de 6000 rpm: com redução de 18,8:1 dá 319 rpm,
// 0,1 A livre, 1,2 A e 1,9 kgf.cm travado; com 6,25:1 dá 960 rpm e 0,6 kgf.cm. O motor de
// 1000 rpm é o mesmo motor com redução de 6:1, e o torque cai na mesma proporção da redução
// (1,9 x 6 / 18,8 = 0,6 kgf.cm). A inércia no eixo da roda soma a do rotor (cerca de 2,5 g.cm²
// num motor do tamanho 520) multiplicada pelo quadrado da redução e a do cubo com o pneu.
// Cubo e pneu: anel de silicone de 10 g entre 12,5 e 16 mm de raio (2,1e-6 kg.m²) e cubo de PLA
// (0,4e-6 kg.m²).
Motor motor319()  { return {"JGB37-520 319 rpm", 12.0, 319, 0.10, 1.2, 1.9 * KGFCM, 2.5e-7 * 18.8 * 18.8 + 2.5e-6}; }
// Motor intermediário: redução de 10:1 da mesma tabela, 600 rpm livre, 480 rpm com carga e 1,1 kgf.cm
// travado (RCDrone). É vendido pronto no AliExpress e em lojas de robótica como "JGB37-520 12 V 600 rpm".
Motor motor600()  { return {"JGB37-520 600 rpm", 12.0, 600, 0.10, 1.2, 1.1 * KGFCM, 2.5e-7 * 10.0 * 10.0 + 2.5e-6}; }
Motor motor1000() { return {"JGB37-520 1000 rpm", 12.0, 1000, 0.10, 1.2, 0.6 * KGFCM, 2.5e-7 * 6.0 * 6.0 + 2.5e-6}; }

double kappaRampa(double mu) {
  double a = ANG_RAMPA * M_PI / 180;
  return (std::cos(a) - mu * std::sin(a)) / (std::sin(a) + mu * std::cos(a));
}

void Robo::eixo(double& ex, double& ey) const {
  ex = x - std::cos(th) * p.xCg;
  ey = y - std::sin(th) * p.xCg;
}
void Robo::paraMundo(double bx, double by, double& wx, double& wy) const {
  double ex, ey, c = std::cos(th), s = std::sin(th);
  eixo(ex, ey);
  wx = ex + c * bx - s * by;
  wy = ey + s * bx + c * by;
}
void Robo::paraCorpo(double wx, double wy, double& bx, double& by) const {
  double ex, ey, c = std::cos(th), s = std::sin(th);
  eixo(ex, ey);
  double dx = wx - ex, dy = wy - ey;
  bx = c * dx + s * dy;
  by = -s * dx + c * dy;
}
void cantos(const Robo& rb, double cx[4], double cy[4]) {
  const double bx[4] = {X_PONTA, X_PONTA, X_TRAS, X_TRAS}, by[4] = {Y_LADO, -Y_LADO, -Y_LADO, Y_LADO};
  for (int k = 0; k < 4; k++) rb.paraMundo(bx[k], by[k], cx[k], cy[k]);
}

Mundo::Mundo(uint32_t semente) : rng(semente) {
  for (auto& rb : r) { rb.p.motor = motor319(); }
}

// Carga nos três apoios sem nenhuma força vertical de fora: peso no centro de massa,
// aceleração do passo anterior e força do contato na altura da rampa.
static void cargasBase(const Robo& rb, double& nE, double& nD, double& nS) {
  const Param& p = rb.p;
  const double W = p.massa * G, hc = 0.006;
  double xs = rb.xApoio;
  nS = (W * p.xCg + rb.fcx * hc - p.massa * rb.ax * p.hCg) / xs;
  double soma = W - nS;
  double dif = 2.0 / BITOLA * (-p.massa * rb.ay * p.hCg + hc * rb.fcy);
  nE = 0.5 * (soma + dif);
  nD = 0.5 * (soma - dif);
}

// derivada da carga de cada apoio por newton levantado no ponto (bx, by) do corpo
static void derivadas(const Robo& rb, double bx, double by, double& dE, double& dD, double& dS) {
  dS = -bx / rb.xApoio;
  double soma = -1 - dS;
  double dif = -2 * by / BITOLA;
  dE = 0.5 * (soma + dif);
  dD = 0.5 * (soma - dif);
}

// Força vertical no ponto (bx, by) que zera o primeiro apoio: a partir daí o ponto sobe pela
// rampa do outro robô e a força não cresce mais.
double Mundo::liftMaximo(const Robo& rb, double bx, double by) {
  double nE, nD, nS, dE, dD, dS;
  cargasBase(rb, nE, nD, nS);
  derivadas(rb, bx, by, dE, dD, dS);
  double v = 1e9;
  if (dE < -1e-9) v = std::min(v, std::max(0.0, nE) / -dE);
  if (dD < -1e-9) v = std::min(v, std::max(0.0, nD) / -dD);
  if (dS < -1e-9) v = std::min(v, std::max(0.0, nS) / -dS);
  return v > 1e8 ? 0 : v;
}

void Mundo::cargas(Robo& rb) {
  double nE, nD, nS, dE, dD, dS;
  cargasBase(rb, nE, nD, nS);
  const Lift* ls[2] = {&rb.lift, &rb.prensa};
  for (const Lift* l : ls) {
    if (l->v == 0) continue;
    derivadas(rb, l->x, l->y, dE, dD, dS);
    nE += l->v * dE; nD += l->v * dD; nS += l->v * dS;
  }
  // apoio com carga negativa sai do chão: o resto do peso fica nos outros dois
  if (nS < 0) { double s = nE + nD; if (s > 0) { nE += nS * nE / s; nD += nS * nD / s; } nS = 0; }
  if (nE < 0) { nD += nE; nE = 0; }
  if (nD < 0) { nE += nD; nD = 0; }
  rb.nE = std::max(0.0, nE); rb.nD = std::max(0.0, nD); rb.nS = std::max(0.0, nS);
  // inclinação só para o desenho: o ponto levantado sobe até 10 mm pela rampa
  rb.arfagem = rb.rolagem = 0;
  if (rb.lift.v > 0) {
    double vMax = std::max(1e-6, liftMaximo(rb, rb.lift.x, rb.lift.y));
    double h = 0.010 * std::min(1.0, rb.lift.v / vMax);
    double d2 = sq(rb.lift.x) + sq(rb.lift.y) + 1e-6;
    rb.arfagem = std::atan(h * rb.lift.x / d2);
    rb.rolagem = std::atan(h * rb.lift.y / d2);
  }
}

void Mundo::ajustaApoioInicial(Robo& rb) {
  rb.ax = rb.ay = rb.fcx = rb.fcy = 0;
  rb.xApoio = X_PONTA;
  rb.lift = rb.prensa = Lift();
  cargas(rb);
  rb.nS0 = rb.nS;
}

// ==================== contato entre os robôs ====================
// Caixas orientadas: eixo de menor penetração (teorema do eixo separador) e corte da face de
// incidência contra a face de referência, como no Box2D, para achar o meio da região de contato.
struct Caixa { double cx, cy, ux, uy, vx, vy, hx, hy; };
static Caixa caixa(const Robo& rb) {
  Caixa c;
  double mx = 0.5 * (X_PONTA + X_TRAS);
  rb.paraMundo(mx, 0, c.cx, c.cy);
  c.ux = std::cos(rb.th); c.uy = std::sin(rb.th);
  c.vx = -c.uy; c.vy = c.ux;
  c.hx = 0.5 * (X_PONTA - X_TRAS); c.hy = Y_LADO;
  return c;
}

void Mundo::contatoRobos(double f[2][3]) {
  for (int k = 0; k < 2; k++) { f[k][0] = f[k][1] = f[k][2] = 0; r[k].lift = r[k].prensa = Lift(); r[k].fcx = r[k].fcy = 0; }
  bool antes = contato;
  contato = false; fContato = 0;
  if (r[0].fora || r[1].fora) return;
  Caixa C[2] = {caixa(r[0]), caixa(r[1])};
  double dx = C[1].cx - C[0].cx, dy = C[1].cy - C[0].cy;
  double melhor = 1e9, nX = 0, nY = 0; int dono = 0, eixoK = 0;
  for (int q = 0; q < 4; q++) {
    const Caixa& o = C[q / 2];
    double ex = (q % 2 == 0) ? o.ux : o.vx, ey = (q % 2 == 0) ? o.uy : o.vy;
    double d = dx * ex + dy * ey;
    double e0 = C[0].hx * std::fabs(C[0].ux * ex + C[0].uy * ey) + C[0].hy * std::fabs(C[0].vx * ex + C[0].vy * ey);
    double e1 = C[1].hx * std::fabs(C[1].ux * ex + C[1].uy * ey) + C[1].hy * std::fabs(C[1].vx * ex + C[1].vy * ey);
    double ov = e0 + e1 - std::fabs(d);
    if (ov <= 0) return;
    // prefere o eixo de A quando as penetrações quase empatam (contato de face estável)
    if (ov < melhor - 1e-5) { melhor = ov; double sg = d >= 0 ? 1 : -1; nX = ex * sg; nY = ey * sg; dono = q / 2; eixoK = q % 2; }
  }
  contato = true;
  // face de referência: do dono do eixo, virada para o outro; incidente: a face do outro mais
  // contrária a ela
  const Caixa& R = C[dono]; const Caixa& I = C[1 - dono];
  double rn[2] = {dono == 0 ? nX : -nX, dono == 0 ? nY : -nY};    // normal da face de referência
  double rcX = R.cx + rn[0] * (eixoK == 0 ? R.hx : R.hy), rcY = R.cy + rn[1] * (eixoK == 0 ? R.hx : R.hy);
  double tX = -rn[1], tY = rn[0];
  double meiaFace = eixoK == 0 ? R.hy : R.hx;
  // face incidente
  double cand[4][2] = {{I.ux, I.uy}, {-I.ux, -I.uy}, {I.vx, I.vy}, {-I.vx, -I.vy}};
  int fi = 0; double dmin = 1e9;
  for (int q = 0; q < 4; q++) { double d = cand[q][0] * rn[0] + cand[q][1] * rn[1]; if (d < dmin) { dmin = d; fi = q; } }
  double fx = cand[fi][0], fy = cand[fi][1];
  double hN = fi < 2 ? I.hx : I.hy, hT = fi < 2 ? I.hy : I.hx;
  double tIx = -fy, tIy = fx;
  double p1x = I.cx + fx * hN + tIx * hT, p1y = I.cy + fy * hN + tIy * hT;
  double p2x = I.cx + fx * hN - tIx * hT, p2y = I.cy + fy * hN - tIy * hT;
  // corta o segmento p1-p2 nas laterais da face de referência
  double s1 = (p1x - rcX) * tX + (p1y - rcY) * tY, s2 = (p2x - rcX) * tX + (p2y - rcY) * tY;
  auto corta = [&](double lim, double sg) {
    double a = sg * s1 - lim, b = sg * s2 - lim;
    if (a > 0 && b > 0) return;
    if (a > 0) { double u = a / (a - b); p1x += (p2x - p1x) * u; p1y += (p2y - p1y) * u; s1 += (s2 - s1) * u; }
    else if (b > 0) { double u = b / (b - a); p2x += (p1x - p2x) * u; p2y += (p1y - p2y) * u; s2 += (s1 - s2) * u; }
  };
  corta(meiaFace, 1); corta(meiaFace, -1);
  double pen1 = -((p1x - rcX) * rn[0] + (p1y - rcY) * rn[1]), pen2 = -((p2x - rcX) * rn[0] + (p2y - rcY) * rn[1]);
  double px, py;
  if (pen1 > 0 && pen2 > 0) { px = 0.5 * (p1x + p2x); py = 0.5 * (p1y + p2y); }
  else if (pen1 > 0) { px = p1x; py = p1y; }
  else if (pen2 > 0) { px = p2x; py = p2y; }
  else { px = 0.5 * (p1x + p2x); py = 0.5 * (p1y + p2y); }
  // Força normal pelo modelo contínuo de FLORES et al. (2011): F = k.d.(1 + 8(1-e)/(5e) . d'/v0),
  // com a penetração d, a taxa de penetração d' e a velocidade v0 do começo do choque. A energia
  // perdida no choque bate com o coeficiente de restituição e, a força começa em zero (sem o
  // degrau da mola amortecida linear) e nunca puxa. A velocidade v0 tem mínimo de 0,3 m/s, para
  // o encosto lento não virar um amortecedor duro demais para o passo de 0,125 ms.
  Robo& A = r[0]; Robo& B = r[1];
  double rAx = px - A.x, rAy = py - A.y, rBx = px - B.x, rBy = py - B.y;
  double vAx = A.vx - A.w * rAy, vAy = A.vy + A.w * rAx;
  double vBx = B.vx - B.w * rBy, vBy = B.vy + B.w * rBx;
  double vrx = vBx - vAx, vry = vBy - vAy;
  double vn = vrx * nX + vry * nY;
  if (!antes) vImpacto = std::max(0.3, -vn);
  const double k = A.p.kContato, e = std::max(0.05, std::min(0.95, A.p.restit));
  double chi = 8 * (1 - e) / (5 * e) / vImpacto;
  // amortecimento k.d.chi limitado para a integração explícita continuar estável (massa
  // reduzida de 0,49 kg)
  double cAm = std::min(k * melhor * chi, 0.5 * 0.49 / 1.25e-4);
  double Fn = std::max(0.0, k * melhor - cAm * vn);
  double ttX = -nY, ttY = nX;
  double vt = vrx * ttX + vry * ttY;
  double Ft = -A.p.muPP * Fn * std::tanh(vt / 0.01);
  double Fx = Fn * nX + Ft * ttX, Fy = Fn * nY + Ft * ttY;      // força em B
  f[1][0] = Fx; f[1][1] = Fy; f[1][2] = rBx * Fy - rBy * Fx;
  f[0][0] = -Fx; f[0][1] = -Fy; f[0][2] = -(rAx * Fy - rAy * Fx);
  fContato = Fn;
  for (int q = 0; q < 2; q++) {
    Robo& rb = r[q];
    double gx = q == 1 ? Fx : -Fx, gy = q == 1 ? Fy : -Fy, cth = std::cos(rb.th), sth = std::sin(rb.th);
    rb.fcx = cth * gx + sth * gy; rb.fcy = -sth * gx + cth * gy;
  }
  // rampas: quem encosta com a frente entra por baixo da lateral ou da traseira do outro; com as
  // duas frentes, a ponta mais baixa vence (altura de impressão mais a ponta que sobe quando o
  // robô empurra e alivia a carga na rampa)
  double pAx, pAy, pBx, pBy;
  A.paraCorpo(px, py, pAx, pAy);
  B.paraCorpo(px, py, pBx, pBy);
  bool frenteA = pAx > X_PONTA - 0.010 && std::cos(A.th) * nX + std::sin(A.th) * nY > 0.5;
  bool frenteB = pBx > X_PONTA - 0.010 && -(std::cos(B.th) * nX + std::sin(B.th) * nY) > 0.5;
  auto pneu = [](double bx, double by) { return std::fabs(by) > Y_PNEU_IN && std::fabs(bx) < X_PNEU + 0.004; };
  double kap = kappaRampa(A.p.muPP);
  auto levanta = [&](Robo& sob, double sx, double sy, Robo& baixo, double bx, double by) {
    double v = std::min(kap * Fn, liftMaximo(sob, sx, sy));
    if (v <= 0) return;
    sob.lift = {v, sx, sy};
    baixo.prensa = {-v, bx, by};
  };
  if (frenteA && frenteB) {
    double hA = A.p.ponta + 1.2e-3 * std::max(0.0, 1 - A.nS / std::max(1e-6, A.nS0));
    double hB = B.p.ponta + 1.2e-3 * std::max(0.0, 1 - B.nS / std::max(1e-6, B.nS0));
    if (hA + 0.15e-3 < hB) levanta(B, X_PONTA, pBy, A, pAx, pAy);
    else if (hB + 0.15e-3 < hA) levanta(A, X_PONTA, pAy, B, pBx, pBy);
  } else if (frenteA && !pneu(pBx, pBy)) {
    levanta(B, pBx, pBy, A, pAx, pAy);
  } else if (frenteB && !pneu(pAx, pAy)) {
    levanta(A, pAx, pAy, B, pBx, pBy);
  }
}

// ==================== pneus, rampa e motores ====================
void Mundo::forcasRobo(Robo& rb, double& fx, double& fy, double& tq, double dt) {
  const Param& p = rb.p;
  const Motor& m = p.motor;
  fx = fy = tq = 0;
  if (rb.fora) return;
  const double c = std::cos(rb.th), s = std::sin(rb.th);
  const double Rm = m.vNom / m.iTravado;
  const double s0 = 0.004;
  // bateria: corrente que sai dela no passo anterior
  double iBat = rb.dutyE * rb.iE + rb.dutyD * rb.iD;
  rb.vBat = std::max(6.0, p.vAberta - p.rBat * iBat);
  double vDisp = rb.vBat - p.queda;
  for (int k = 0; k < 2; k++) {
    const double wLivre = m.rpmLivre * (1 + p.varRpm[k]) * 2 * M_PI / 60;
    const double kE = (m.vNom - m.iLivre * Rm) / wLivre;
    const double kT = m.torqueTravado * (1 + p.varTorque[k]) / m.iTravado;
    const double tFric = kT * m.iLivre;
    double by = (k == 0 ? 1 : -1) * BITOLA / 2;
    double wxp, wyp;
    rb.paraMundo(0, by, wxp, wyp);
    double rx = wxp - rb.x, ry = wyp - rb.y;
    double vpx = rb.vx - rb.w * ry, vpy = rb.vy + rb.w * rx;
    double vl = c * vpx + s * vpy, vt = -s * vpx + c * vpy;
    double& wr = k == 0 ? rb.wE : rb.wD;
    double N = k == 0 ? rb.nE : rb.nD;
    double sl = vl - wr * R_RODA, st = vt;
    double sm = std::sqrt(sl * sl + st * st);
    double mu = p.muDin + (p.muEst - p.muDin) * std::exp(-sq(sm / p.vStrib));
    double den = std::sqrt(sm * sm + s0 * s0);
    double fl = -mu * N * sl / den - p.cRol * N * std::tanh(vl / 0.01);
    double ft = -mu * N * st / den;
    // motor: tensão média do PWM (ponte com freio no tempo desligado) e corrente
    double duty = k == 0 ? rb.dutyE : rb.dutyD;
    double V = duty * vDisp;
    double i = (V - kE * wr) / Rm;
    double tm = kT * i - tFric * std::tanh(wr / 1.0);
    // roda semi-implícita: o atrito com o chão e a força contraeletromotriz entram como
    // amortecimento para o passo ficar estável com o pneu grudando
    double amort = R_RODA * R_RODA * mu * N / den + kT * kE / Rm;
    wr += dt * (tm + R_RODA * mu * N * sl / den) / (m.jRoda + dt * amort);
    (k == 0 ? rb.iE : rb.iD) = i;
    (k == 0 ? rb.angE : rb.angD) += wr * dt;
    (k == 0 ? rb.fTracE : rb.fTracD) = fl;
    double Fx = fl * c - ft * s, Fy = fl * s + ft * c;
    fx += Fx; fy += Fy; tq += rx * Fy - ry * Fx;
  }
  // ponta da rampa arrastando no dojô
  double wxp, wyp;
  rb.paraMundo(rb.xApoio, 0, wxp, wyp);
  double rx = wxp - rb.x, ry = wyp - rb.y;
  double vpx = rb.vx - rb.w * ry, vpy = rb.vy + rb.w * rx;
  double vm = std::sqrt(vpx * vpx + vpy * vpy);
  double den = std::sqrt(vm * vm + s0 * s0);
  double Fx = -p.muPla * rb.nS * vpx / den, Fy = -p.muPla * rb.nS * vpy / den;
  fx += Fx; fy += Fy; tq += rx * Fy - ry * Fx;
  // arrasto do ar no centro de massa
  double va = std::hypot(rb.vx, rb.vy), kAr = 0.5 * RHO_AR * CD_AR * A_AR * va;
  fx -= kAr * rb.vx; fy -= kAr * rb.vy;
}

void Mundo::integra(Robo& rb, double fx, double fy, double tq, double dt) {
  if (rb.fora) { rb.vx = rb.vy = rb.w = 0; return; }
  double axw = fx / rb.p.massa, ayw = fy / rb.p.massa;
  rb.vx += axw * dt; rb.vy += ayw * dt;
  rb.w  += tq / rb.p.izz * dt;
  rb.x += rb.vx * dt; rb.y += rb.vy * dt; rb.th += rb.w * dt;
  if (rb.th > M_PI) rb.th -= 2 * M_PI;
  if (rb.th < -M_PI) rb.th += 2 * M_PI;
  // aceleração no corpo, filtrada, para a transferência de carga do próximo passo
  double c = std::cos(rb.th), s = std::sin(rb.th);
  double axb = c * axw + s * ayw, ayb = -s * axw + c * ayw;
  const double a = 0.05;
  rb.ax += a * (axb - rb.ax); rb.ay += a * (ayb - rb.ay);
}

// O robô sai quando encosta no chão de fora: uma roda passa da borda, o centro de massa passa
// da borda ou a ponta passa da borda e a quina do dojô fica atrás do centro de massa (o robô
// tomba para a frente). Com a ponta para fora e o centro de massa dentro, o corpo apoia na
// quina e o robô continua no dojô.
void Mundo::confereFora(Robo& rb) {
  if (rb.fora) return;
  int motivo = 0;
  for (int k = 0; k < 2 && !motivo; k++) {
    double wx, wy;
    rb.paraMundo(0, (k ? -1 : 1) * BITOLA / 2, wx, wy);
    if (std::hypot(wx, wy) > R_DOJO) motivo = 1;
  }
  if (!motivo && std::hypot(rb.x, rb.y) > R_DOJO) motivo = 2;
  // apoio da frente: ponta da rampa ou quina do dojô na linha de centro do robô
  double ex, ey, ux = std::cos(rb.th), uy = std::sin(rb.th);
  rb.eixo(ex, ey);
  double tx, ty;
  rb.paraMundo(X_PONTA, 0, tx, ty);
  if (std::hypot(tx, ty) > R_DOJO) {
    double b = ex * ux + ey * uy, cc = ex * ex + ey * ey - R_DOJO * R_DOJO;
    double disc = b * b - cc;
    double sEdge = disc > 0 ? -b + std::sqrt(disc) : 0;
    rb.xApoio = std::max(0.005, std::min(X_PONTA, sEdge));
    if (!motivo && rb.xApoio < rb.p.xCg) motivo = 3;
  } else rb.xApoio = X_PONTA;
  if (motivo) { rb.fora = true; rb.tFora = t; rb.motivoFora = motivo; }
}

void Mundo::passo(double dt) {
  if (motorFisico) {
    motorFisico(*this, dt);
    t += dt;
    return;
  }
  double f[2][3];
  contatoRobos(f);
  if (contato) { tUltContato = t; tContatoTotal += dt; }
  double g[2][3];
  for (int k = 0; k < 2; k++) cargas(r[k]);
  for (int k = 0; k < 2; k++) forcasRobo(r[k], g[k][0], g[k][1], g[k][2], dt);
  for (int k = 0; k < 2; k++) integra(r[k], g[k][0] + f[k][0], g[k][1] + f[k][1], g[k][2] + f[k][2], dt);
  t += dt;
  for (int k = 0; k < 2; k++) confereFora(r[k]);
}

// ==================== sensores ====================
// TCRT5000 a 4 mm do chão (bancada, 01/10): preto perto de 1300 mV, branco perto de 260 mV e,
// depois da borda (chão 2,5 cm mais baixo), quase nada volta e a leitura fica alta. As linhas
// Shikiri marrons absorvem o infravermelho como o preto.
double Mundo::bordaMv(int k, double bxmm, double bymm) {
  double wx, wy;
  r[k].paraMundo(bxmm / 1000, bymm / 1000, wx, wy);
  double d = std::hypot(wx, wy), mv;
  if (d > R_DOJO)              mv = 2400 + normal(80);
  else if (d > R_DOJO - FAIXA) mv = 260 + normal(30);
  else                         mv = 1300 + normal(60);
  if (uniforme(0, 1) < arranhoes) mv = uniforme(0, 1) < 0.5 ? 300 : 700;
  return std::max(0.0, std::min(3300.0, mv));
}

static double raioSegmento(double ox, double oy, double dx, double dy, double ax, double ay, double bx, double by, double& cosInc) {
  double ex = bx - ax, ey = by - ay;
  double den = dx * ey - dy * ex;
  if (std::fabs(den) < 1e-12) return -1;
  double t = ((ax - ox) * ey - (ay - oy) * ex) / den;
  double u = ((ax - ox) * dy - (ay - oy) * dx) / den;
  if (t <= 0 || u < 0 || u > 1) return -1;
  cosInc = std::fabs(den / std::hypot(ex, ey));
  return t;
}

// Faces do robô vistas por um ToF na altura das lentes: face da frente inclinada (61,8 graus,
// o infravermelho volta menos), paredes laterais, pneus pretos e, atrás, as caixas de redução
// metálicas e as abraçadeiras.
struct Face { double x1, y1, x2, y2, refl, incl; };
static int facesRobo(const Robo& rb, Face* f) {
  const double re = rb.p.refl, tras = 0.5 * 0.45 + 0.5 * re, pneu = 0.04, ypn = 0.0738;
  int n = 0;
  f[n++] = {X_FACE, -Y_LADO, X_FACE, Y_LADO, re, 0.88};
  for (int sg = -1; sg <= 1; sg += 2) {
    f[n++] = {X_PNEU, sg * Y_LADO, X_FACE, sg * Y_LADO, re, 1};
    f[n++] = {-X_PNEU, sg * ypn, X_PNEU, sg * ypn, pneu, 1};
    f[n++] = {X_TRAS, sg * Y_LADO, -X_PNEU, sg * Y_LADO, tras, 1};
    f[n++] = {-X_PNEU, sg * Y_PNEU_IN, -X_PNEU, sg * ypn, pneu, 1};
  }
  f[n++] = {X_TRAS, -Y_PNEU_IN, X_TRAS, Y_PNEU_IN, tras, 1};
  return n;
}

// ToF: 9 raios no cone de 25 graus. Cada objeto atingido soma sinal (refletância, incidência e
// 1/d²); o mais forte define a leitura, e sinal abaixo do limiar (com flutuação) vira 8190.
int Mundo::tofMede(int k, double sxmm, double symm, double angGraus, const PerfilToF& pf) {
  const Robo& eu = r[k]; const Robo& op = r[1 - k];
  double wx, wy;
  eu.paraMundo(sxmm / 1000, symm / 1000, wx, wy);
  Face fc[12]; int nf = 0;
  Face fw[12];
  if (!op.fora) {
    nf = facesRobo(op, fc);
    for (int q = 0; q < nf; q++) {
      op.paraMundo(fc[q].x1, fc[q].y1, fw[q].x1, fw[q].y1);
      op.paraMundo(fc[q].x2, fc[q].y2, fw[q].x2, fw[q].y2);
      fw[q].refl = fc[q].refl; fw[q].incl = fc[q].incl;
    }
  }
  const int NR = 9, NOBJ = 3 + 8;           // 0 oponente, 1 parede, 2.. pessoas
  double sinal[NOBJ] = {0}, dist[NOBJ] = {0}, peso[NOBJ] = {0};
  double pesoTotal = 0;
  const double pa[4][4] = {{-salaX, -salaY, salaX, -salaY}, {salaX, -salaY, salaX, salaY},
                           {salaX, salaY, -salaX, salaY}, {-salaX, salaY, -salaX, -salaY}};
  for (int q = 0; q < NR; q++) {
    double frac = (q - (NR - 1) / 2.0) / ((NR - 1) / 2.0);
    double wR = std::exp(-1.5 * frac * frac);
    pesoTotal += wR;
    double a = eu.th + (angGraus + 12.5 * frac) * M_PI / 180;
    double dx = std::cos(a), dy = std::sin(a);
    double tMin = 1e9, cMin = 1, rMin = 0; int oMin = -1;
    for (int e = 0; e < nf; e++) {
      double ci, t = raioSegmento(wx, wy, dx, dy, fw[e].x1, fw[e].y1, fw[e].x2, fw[e].y2, ci);
      if (t > 0 && t < tMin) { tMin = t; cMin = ci * fw[e].incl; rMin = fw[e].refl; oMin = 0; }
    }
    for (int e = 0; e < 4; e++) {
      double ci, t = raioSegmento(wx, wy, dx, dy, pa[e][0], pa[e][1], pa[e][2], pa[e][3], ci);
      // metade do feixe bate no chão antes de chegar à parede
      if (t > 0 && t < tMin) { tMin = t; cMin = 0.5 * ci; rMin = reflParede; oMin = 1; }
    }
    for (size_t j = 0; j < pessoas.size() && j < 8; j++) {
      const Pessoa& o = pessoas[j];
      double fx = wx - o.x, fy = wy - o.y;
      double b = fx * dx + fy * dy, cc = fx * fx + fy * fy - o.raio * o.raio;
      double disc = b * b - cc;
      if (disc < 0) continue;
      double t = -b - std::sqrt(disc);
      if (t > 0 && t < tMin) { tMin = t; cMin = 1; rMin = o.refl; oMin = 2 + (int)j; }
    }
    if (oMin < 0) continue;
    double d10 = tMin / 0.1;
    sinal[oMin] += rMin * cMin * wR / (d10 * d10);
    dist[oMin] += tMin * wR;
    peso[oMin] += wR;
  }
  int melhor = -1; double sMax = 0;
  for (int o = 0; o < NOBJ; o++) if (sinal[o] > sMax) { sMax = sinal[o]; melhor = o; }
  if (uniforme(0, 1) < pf.falsa) return (int)uniforme(30, 900);
  if (melhor < 0) return 8190;
  double lim = pf.limiar * ambienteToF;
  double sn = sMax / pesoTotal * std::exp(normal(0.2));
  if (sn < lim) return 8190;
  double d = dist[melhor] / peso[melhor] * 1000;
  if (d > pf.alcanceMax) return 8190;
  double sg = pf.ruido * (4 + 0.015 * d) * (1 + lim / sn);
  int mm = (int)std::lround(d + normal(sg));
  return mm < 1 ? 1 : mm;
}

}  // namespace cb
