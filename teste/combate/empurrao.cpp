// empurrao.cpp
// Empurrão frente a frente, em malha aberta (sem firmware): os dois robôs de frente um para o
// outro, a 40 cm, com o PWM em 100% até alguém sair do dojô. O robô A é sempre o do Grupo D (motor
// de 319 rpm) e o B é o adversário. Serve para comparar as duas físicas do simulador de combate no
// caso mais simples, o do empurrão, onde a rampa de um robô entra por baixo da lâmina do outro.
//
// Uso: empurrao [-F simples|mujoco] [-m RPM_B] [-n LUTAS] [-s SEMENTE] [-t SEGUNDOS]
//   -F  física: simples (contas à mão do fisica.cpp, padrão) ou mujoco (precisa de MUJOCO=1 no compila.sh)
//   -m  motor do adversário: 1000, 600 ou 319 (padrão)
//   -n  quantidade de empurrões (padrão 50); cada um sorteia o desalinhamento lateral (±30 mm), o
//       giro (±3°) e o lado de largada de cada robô
//   -t  tempo máximo de cada empurrão (padrão 15 s)
// Saída: CSV com uma linha por empurrão (cabeçalho com -H).
//   vencedor: 0 Grupo D, 1 adversário, 2 os dois, -1 sem decisão no tempo
//   arf_a e arf_b: maior arfagem de cada robô em graus (o robô que fica com o nariz levantado)
#include "fisica.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace cb;

int main(int argc, char** argv) {
  bool mj = false;
  int rpmB = 319, n = 50;
  uint32_t sem = 1;
  double tMax = 15;
  for (int i = 1; i < argc; i++) {
    auto prox = [&]() { return i + 1 < argc ? argv[++i] : (char*)""; };
    if (!strcmp(argv[i], "-F")) mj = !strcmp(prox(), "mujoco");
    else if (!strcmp(argv[i], "-m")) rpmB = atoi(prox());
    else if (!strcmp(argv[i], "-n")) n = atoi(prox());
    else if (!strcmp(argv[i], "-s")) sem = (uint32_t)atol(prox());
    else if (!strcmp(argv[i], "-t")) tMax = atof(prox());
    else if (!strcmp(argv[i], "-H")) { puts("lut,fisica,rpm_b,desalinho_mm,giro_graus,lado_a,vencedor,tempo,arf_a,arf_b,f_max"); return 0; }
  }
  for (int q = 0; q < n; q++) {
    Mundo M(sem * 7919u + q);
    M.arranhoes = 0;
    const double desal = M.uniforme(-0.030, 0.030), giro = M.uniforme(-3, 3) * M_PI / 180;
    const int lado = M.uniforme(0, 1) < 0.5 ? 1 : -1;     // lado de largada do robô A
    for (int k = 0; k < 2; k++) {
      Robo& rb = M.r[k];
      rb.p.motor = k == 1 ? (rpmB == 1000 ? motor1000() : rpmB == 600 ? motor600() : motor319()) : motor319();
      rb.p.ponta = M.uniforme(0.0, 1.0) * 1e-3;
      const double sg = k == 0 ? lado : -lado;           // posição em x do eixo: sg * 0,20 m
      const double th = (sg > 0 ? M_PI : 0) + (k == 1 ? giro : 0);
      const double ex = sg * 0.20, ey = k == 1 ? desal : 0;
      rb.th = th; rb.x = ex + std::cos(th) * rb.p.xCg; rb.y = ey + std::sin(th) * rb.p.xCg;
      rb.dutyE = rb.dutyD = 1.0;
      M.ajustaApoioInicial(rb);
    }
    if (mj) mujocoLiga(M);
    double fMax = 0, arf[2] = {0, 0};
    int ven = -1;
    while (M.t < tMax) {
      for (int k = 0; k < 2; k++) M.r[k].dutyE = M.r[k].dutyD = 1;
      M.passo(0.125e-3);
      fMax = std::max(fMax, M.fContato);
      for (int k = 0; k < 2; k++) arf[k] = std::max(arf[k], std::fabs(M.r[k].arfagem));
      if (M.r[0].fora || M.r[1].fora) { ven = M.r[0].fora && M.r[1].fora ? 2 : (M.r[1].fora ? 0 : 1); break; }
    }
    const double tFim = ven == 0 ? M.r[1].tFora : ven == 1 ? M.r[0].tFora : M.t;
    printf("%d,%s,%d,%.1f,%.2f,%d,%d,%.2f,%.1f,%.1f,%.1f\n", q, mj ? "mujoco" : "simples", rpmB, desal * 1000,
           giro * 180 / M_PI, lado, ven, tFim, arf[0] * 180 / M_PI, arf[1] * 180 / M_PI, fMax);
  }
  return 0;
}
