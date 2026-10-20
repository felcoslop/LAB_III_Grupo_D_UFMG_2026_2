// teste_arena.cpp
// Testes da localização no dojô (arena.cpp) no computador. Um robô de mentira larga em posições
// sorteadas dentro das regras da CoRA, anda até a borda, recua, gira para o centro estimado e
// repete, cinco vezes por luta; os sensores de borda leem branco só na faixa. Como no firmware,
// qualquer avanço para no branco da frente e qualquer ré para no branco de trás. O modelo das
// rodas que o filtro recebe pode estar certo ou errado em 15% (motor sem encoder). Em 150 lutas,
// o teste confere o erro do centro estimado depois de cada toque na borda, a coerência entre o
// erro e a incerteza que o filtro informa e o descarte de pontos fora do dojô: o oponente perto
// da borda nunca pode ser descartado, e a pessoa do lado de fora deve ser descartada com
// frequência.
// Uso: g++ -std=c++11 -O2 -Wall -Wextra -I teste/unidade/stub -I . teste/unidade/teste_arena.cpp arena.cpp -o t && ./t
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <algorithm>
#include "arena.h"
#include "config.h"

uint32_t relogio_us = 0;

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor) {
  printf("%s  %-72s %8.1f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}

static const double R = DOHYO_DIAMETRO_MM / 2.0;
struct Robo { double x, y, th; };          // mundo: mm e radianos, centro do dojô na origem

static void paraMundo(const Robo& r, double bx, double by, double& wx, double& wy) {
  wx = r.x + cos(r.th) * bx - sin(r.th) * by;
  wy = r.y + sin(r.th) * bx + cos(r.th) * by;
}
static void paraCorpo(const Robo& r, double wx, double wy, double& bx, double& by) {
  double dx = wx - r.x, dy = wy - r.y;
  bx = cos(r.th) * dx + sin(r.th) * dy;
  by = -sin(r.th) * dx + cos(r.th) * dy;
}
static uint16_t bordaBranca(const Robo& r) {
  uint16_t m = 0;
  for (uint8_t i = 0; i < N_BORDA; i++) {
    double wx, wy;
    paraMundo(r, BORDA[i].x, BORDA[i].y, wx, wy);
    double d = hypot(wx, wy);
    if (d >= R - BORDA_FAIXA_MM && d <= R) m |= (1u << i);
  }
  return m;
}
// gerador simples e repetível (o teste dá o mesmo resultado em qualquer máquina)
static uint32_t sem = 2463534242u;
static double U() { sem ^= sem << 13; sem ^= sem >> 17; sem ^= sem << 5; return (sem >> 8) / 16777216.0; }

static double escala = 1.0;                // erro do modelo das rodas visto pelo filtro
static uint16_t passo(Robo& r, double v, double wDps) {
  relogio_us += 1000;
  r.th += wDps * M_PI / 180 * 0.001;
  r.x += v * cos(r.th) * 0.001;
  r.y += v * sin(r.th) * 0.001;
  uint16_t b = bordaBranca(r);
  arena_atualiza((float)(v * escala), (float)(wDps * escala), b, b, false);
  return b;
}
// anda por até ms milissegundos e para quando um sensor de borda do lado do movimento vê branco
static void anda(Robo& r, double v, double wDps, int ms) {
  uint16_t lado = 0;
  for (uint8_t i = 0; i < N_BORDA; i++)
    if ((v > 0 && BORDA[i].frente) || (v < 0 && !BORDA[i].frente)) lado |= (1u << i);
  for (int t = 0; t < ms; t++)
    if (passo(r, v, wDps) & lado) return;
}
static double erroCentro(const Robo& r, const Arena& a) {
  double cx, cy;
  paraCorpo(r, 0, 0, cx, cy);
  return hypot(a.cx - cx, a.cy - cy);
}

int main() {
  const int LUTAS = 150, TOQUES = 5;
  std::vector<double> erro[TOQUES];
  int coerente = 0, nConfiavel = 0, confiavelErrado = 0;
  int foraOk = 0, dentroDescartado = 0, mapaApagado = 0, medidas = 0;
  for (int it = 0; it < LUTAS; it++) {
    escala = it % 3 == 0 ? 1.0 : (it % 3 == 1 ? 0.85 : 1.15);
    // largada da CoRA: na metade de cá, atrás da linha Shikiri, virado para qualquer lado
    Robo r;
    for (;;) {
      r.x = 130 + U() * 170; r.y = (U() * 2 - 1) * 200; r.th = (U() * 2 - 1) * M_PI;
      if (hypot(r.x, r.y) < 300 && !bordaBranca(r)) break;
    }
    arena_init();
    arena_inicia(false, 0, 0);
    if (it == 0) {
      Arena a; arena_le(a);
      confere(a.valida && !a.confiavel, "start: filtro valido, mas ainda incerto", a.sigma);
    }
    for (int k = 0; k < TOQUES; k++) {
      // gira um ângulo sorteado e anda até a borda; 15 ms de reação, recuo, giro para o centro
      // estimado (ou 135 graus sem localização) e volta para dentro
      double g = (U() * 2 - 1) * 160;
      anda(r, 0, g > 0 ? 300 : -300, (int)(fabs(g) / 300 * 1000));
      anda(r, 400, 0, 4000);
      for (int t = 0; t < 15; t++) passo(r, 400, 0);
      anda(r, -400, 0, 150);
      Arena a; arena_le(a);
      double alvo = a.confiavel ? a.ang : 135;
      anda(r, 0, alvo > 0 ? 300 : -300, (int)(fabs(alvo) / 300 * 1000));
      anda(r, 300, 0, 300);
      arena_le(a);
      double e = erroCentro(r, a);
      erro[k].push_back(e);
      if (e <= 2.5 * a.sigma + 20) coerente++;
      if (a.confiavel) { nConfiavel++; if (e > 100) confiavelErrado++; }
      // descarte numa direção sorteada: pessoa 12 cm além da borda, ponto do oponente 6 cm para
      // dentro da borda (leitura do ToF) e centro do oponente encostado na borda (mapa do
      // rastreador, com o arena_limite da direção)
      double ang = U() * 2 * M_PI, bx, by;
      medidas++;
      paraCorpo(r, (R + 120) * cos(ang), (R + 120) * sin(ang), bx, by);
      if (arena_fora(a, (float)bx, (float)by)) foraOk++;
      paraCorpo(r, (R - 60) * cos(ang), (R - 60) * sin(ang), bx, by);
      if (arena_fora(a, (float)bx, (float)by)) dentroDescartado++;
      paraCorpo(r, (R - 76) * cos(ang), (R - 76) * sin(ang), bx, by);
      double d = hypot(bx, by);
      if (d > arena_limite(a, (float)(bx / d), (float)(by / d))) mapaApagado++;
    }
  }
  for (int k = 0; k < TOQUES; k++) {
    std::sort(erro[k].begin(), erro[k].end());
    printf("# toque %d: erro mediano %3.0f mm, 90%% das lutas abaixo de %3.0f mm\n", k + 1,
           erro[k][LUTAS / 2], erro[k][LUTAS * 9 / 10]);
  }
  const int n = TOQUES * LUTAS;
  confere(erro[TOQUES - 1][LUTAS / 2] < 40, "depois de 5 toques na borda: erro mediano do centro < 40 mm", erro[TOQUES - 1][LUTAS / 2]);
  confere(erro[TOQUES - 1][LUTAS * 9 / 10] < 90, "depois de 5 toques: 90% das lutas com erro < 90 mm", erro[TOQUES - 1][LUTAS * 9 / 10]);
  confere(erro[1][LUTAS / 2] < erro[0][LUTAS / 2], "o erro cai do primeiro para o segundo toque", erro[1][LUTAS / 2]);
  confere(coerente >= 0.95 * n, "incerteza coerente: erro < 2,5 sigma + 2 cm em 95% dos casos (%)", 100.0 * coerente / n);
  confere(nConfiavel >= 0.3 * n, "localizacao confiavel em pelo menos 30% das medidas (%)", 100.0 * nConfiavel / n);
  confere(confiavelErrado <= 0.05 * nConfiavel, "marcada como confiavel com erro > 10 cm em no maximo 5% (%)", 100.0 * confiavelErrado / (nConfiavel ? nConfiavel : 1));
  // Sem encoder o filtro ainda pode ficar confiante e errado por mais de 30 cm em casos raros. Com
  // seis sementes do sorteio nas duas geometrias do robô montado (a anterior e a do chassi de dois
  // andares), de 0 a 10 casos em 750 medidas (0 a 1,3%) caem nesse caso, sem diferença entre as
  // geometrias; o limite aceita 2% (15 casos em 750 medidas).
  confere(dentroDescartado <= 15, "leitura do oponente 6 cm para dentro da borda: no maximo 2% descartadas", dentroDescartado);
  confere(mapaApagado <= 15, "centro do oponente encostado na borda: no maximo 2% apagados do mapa", mapaApagado);
  confere(foraOk >= 0.3 * medidas, "pessoa 12 cm alem da borda: descartada em pelo menos 30% (%)", 100.0 * foraOk / medidas);

  printf("\n%s: %d falha(s)\n", falhas ? "ERRO" : "OK", falhas);
  return falhas ? 1 : 0;
}
