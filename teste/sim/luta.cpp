// luta.cpp
// Simulador de luta com o firmware de verdade (software in the loop). O executável compila os
// módulos do firmware (percepção, ToF, rastreador, borda, estados, cinemática e motores) junto
// com o mundo simulado (mundo.cpp) e o Arduino de mentira (hal.cpp), posiciona os dois robôs
// como manda o regulamento da CoRA (cada um na sua metade, atrás da linha Shikiri, virado para
// qualquer lado), aperta o start e roda o loop do firmware a cada 1 ms até alguém sair do dojô
// ou acabar o round de 90 s. No fim imprime uma linha CSV com o resultado.
//
// Uso: luta [opções]
//   -s N        semente (posição inicial, ruído dos sensores)
//   -o TIPO     oponente: parado, pesado, rapido, lento, errante, espelho
//   -c COR      claro, escuro ou furtivo (refletância do oponente para o ToF)
//   -d BITS     chave DIP (bit 0 = DIP 1 ... bit 3 = DIP 4), como número decimal
//   -f          coloca pessoas paradas fora do dojô, visíveis para os ToF
//   -t S        duração do round em segundos (padrão 90)
//   -b V        tensão da bateria dos motores (padrão: sorteada entre 11,4 e 12,6 V)
//   -r ARQ      grava o rastro da luta em JSON (para o visualizador da página)
//   -v          mostra o Log do firmware
//   -a          diagnóstico a cada 100 ms: arena e oponente estimados contra os de verdade
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>
#include "Arduino.h"
#include "config.h"
#include "percepcao.h"
#include "estados.h"
#include "cinematica.h"
#include "motores.h"
#include "hal.h"
#include "mundo.h"

using namespace sim;

// ==================== IA do oponente (com a posição real, atrasada) ====================
struct IA {
  std::string tipo;
  double vMax = 0.5;            // m/s em linha reta
  double tDecide = 0, tFase = 0;
  int    fase = 0;              // 0 normal, 1 recuo da borda, 2 giro da borda
  double giroAlvo = 0;
  double rumoErrante = 0, tRumo = 0;
  double vCmd = 0, wCmd = 0;
  std::vector<double> hx, hy;   // histórico da posição do robô do Grupo D (atraso de percepção)
};

static double normaliza(double a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }

// velocidade livre da roda do oponente, para converter (v, w) em tensão
static double vLivre(const Param& p) { return p.rpmLivre * 2 * M_PI / 60 * p.raio; }

static void iaPasso(Mundo& M, IA& ia, double dt, bool liberado) {
  Robo& o = M.op;
  if (o.p.freado || !liberado || o.fora) { o.rodaE.v = o.rodaD.v = 0; return; }
  ia.hx.push_back(M.nosso.x); ia.hy.push_back(M.nosso.y);
  const size_t atraso = 8;      // 8 passos de 5 ms = 40 ms de atraso de percepção
  if (ia.hx.size() > atraso + 1) { ia.hx.erase(ia.hx.begin()); ia.hy.erase(ia.hy.begin()); }
  double ax = ia.hx.front(), ay = ia.hy.front();
  double dx = ax - o.x, dy = ay - o.y, dist = std::hypot(dx, dy);
  double rumo = normaliza(std::atan2(dy, dx) - o.th);
  const double vm = vLivre(o.p) * 0.95;
  // borda: os cantos da frente ou de trás do oponente na faixa branca
  double fx, fy, tx, ty;
  o.paraMundo(o.p.meioX * 0.8, 0, fx, fy);
  o.paraMundo(-o.p.meioX * 0.8, 0, tx, ty);
  bool bordaFrente = std::hypot(fx, fy) > R_DOJO - FAIXA, bordaTras = std::hypot(tx, ty) > R_DOJO - FAIXA;
  if (ia.fase == 0 && (bordaFrente || bordaTras) && M.t - ia.tFase > 0.3) {
    ia.fase = 1; ia.tFase = M.t; ia.vCmd = bordaFrente ? -vm : vm; ia.wCmd = 0;
  }
  if (ia.fase == 1) {
    if (M.t - ia.tFase > 0.25) { ia.fase = 2; ia.tFase = M.t; ia.giroAlvo = M.uniforme(0, 1) < 0.5 ? 1 : -1; }
  } else if (ia.fase == 2) {
    ia.vCmd = 0; ia.wCmd = ia.giroAlvo * 6;
    if (M.t - ia.tFase > 0.35) { ia.fase = 0; ia.tFase = M.t; }
  } else {
    bool ve = dist < 0.65 && (ia.tipo != "errante" || (dist < 0.35 && std::fabs(rumo) < 0.6));
    if (ve) {
      ia.wCmd = std::max(-8.0, std::min(8.0, 10 * rumo));
      ia.vCmd = std::fabs(rumo) < 0.35 ? vm : (std::fabs(rumo) < 1.0 ? 0.35 * vm : 0);
    } else if (ia.tipo == "errante") {
      if (M.t > ia.tRumo) { ia.rumoErrante = M.uniforme(-M_PI, M_PI); ia.tRumo = M.t + M.uniforme(0.6, 1.5); }
      double alvo = normaliza(ia.rumoErrante - o.th);
      ia.wCmd = std::max(-5.0, std::min(5.0, 6 * alvo));
      ia.vCmd = 0.5 * vm;
    } else {
      ia.vCmd = 0; ia.wCmd = 3.5;     // procura girando
    }
  }
  // diferencial: tensão proporcional à velocidade pedida de cada roda
  double vE = ia.vCmd - ia.wCmd * o.p.bitola / 2, vD = ia.vCmd + ia.wCmd * o.p.bitola / 2;
  double vMaxRoda = vLivre(o.p);
  double esc = std::max(1.0, std::max(std::fabs(vE), std::fabs(vD)) / vMaxRoda);
  o.rodaE.v = vE / esc / vMaxRoda * o.p.vBat;
  o.rodaD.v = vD / esc / vMaxRoda * o.p.vBat;
  (void)dt;
}

// ==================== posição inicial (regulamento da CoRA) ====================
// Cada robô fica inteiro na sua metade, atrás da linha imaginária da Shikiri (a 5 cm do centro),
// com folga para nenhum sensor de borda começar no branco. O rumo é um de cinco: virado para o
// oponente, para os dois lados, de costas ou sorteado.
static void posiciona(Mundo& M, Robo& r, int lado, int jeito) {
  for (int tent = 0; tent < 1000; tent++) {
    double th;
    switch (jeito) {
      case 0: th = lado > 0 ? M_PI : 0; break;              // de frente para o oponente
      case 1: th = M_PI / 2; break;
      case 2: th = -M_PI / 2; break;
      case 3: th = lado > 0 ? 0 : M_PI; break;              // de costas
      default: th = M.uniforme(-M_PI, M_PI);
    }
    r.th = th;
    r.x = lado * M.uniforme(0.06, 0.30);
    r.y = M.uniforme(-0.25, 0.25);
    double cx[4], cy[4];
    r.cantos(cx, cy);
    bool ok = true;
    for (int k = 0; k < 4; k++) {
      if (lado * cx[k] < 0.05) ok = false;                          // passou da linha Shikiri
      if (std::hypot(cx[k], cy[k]) > R_DOJO - FAIXA - 0.012) ok = false;   // canto no branco
    }
    if (ok) return;
  }
  r.x = lado * 0.15; r.y = 0; r.th = lado > 0 ? M_PI : 0;
}

// ==================== rastro para o visualizador ====================
static FILE* rastro = nullptr;
static bool primeiroRastro = true;
static void gravaRastro(const Mundo& M) {
  if (!rastro) return;
  float ox = (float)(M.nosso.x * 1000), oy = (float)(M.nosso.y * 1000);
  // estimativa do oponente no referencial do dojô (o firmware guarda no referencial do robô)
  double c = std::cos(M.nosso.th), s = std::sin(M.nosso.th);
  double ex = M.nosso.x * 1000 + c * P.op.x - s * P.op.y, ey = M.nosso.y * 1000 + s * P.op.x + c * P.op.y;
  fprintf(rastro, "%s[%.3f,%.0f,%.0f,%.3f,%.0f,%.0f,%.3f,%d,%d,%.0f,%.0f,%d,%d", primeiroRastro ? "" : ",\n",
          M.t, ox, oy, M.nosso.th, M.op.x * 1000, M.op.y * 1000, M.op.th, (int)estado_atual(),
          P.op.visto ? 1 : 0, ex, ey, motores_esq(), motores_dir());
#if __has_include("arena.h")
  // centro do dojô estimado (arena.cpp) no referencial do dojô, incerteza e confiança
  if (P.arena.valida) {
    double ax = M.nosso.x * 1000 + c * P.arena.cx - s * P.arena.cy, ay = M.nosso.y * 1000 + s * P.arena.cx + c * P.arena.cy;
    fprintf(rastro, ",%.0f,%.0f,%.0f,%d", ax, ay, P.arena.sigma, P.arena.confiavel ? 1 : 0);
  } else fprintf(rastro, ",0,0,-1,0");
#endif
  fprintf(rastro, "]");
  primeiroRastro = false;
}

int main(int argc, char** argv) {
  uint32_t semente = 1;
  std::string tipo = "espelho", cor = "escuro", arqRastro;
  int dip = 0;
  bool pessoas = false, diag = false;
  double tRound = 90, vBat = -1;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto prox = [&]() { return i + 1 < argc ? argv[++i] : (char*)""; };
    if (a == "-s") semente = (uint32_t)atol(prox());
    else if (a == "-o") tipo = prox();
    else if (a == "-c") cor = prox();
    else if (a == "-d") dip = atoi(prox());
    else if (a == "-f") pessoas = true;
    else if (a == "-t") tRound = atof(prox());
    else if (a == "-b") vBat = atof(prox());
    else if (a == "-r") arqRastro = prox();
    else if (a == "-v") H.verboso = true;
    else if (a == "-a") diag = true;
  }
  Mundo M(semente * 2654435761u + 17);
  H.mundo = &M;
  H.dip = (uint8_t)dip;
  M.op.p = paramOponente(tipo);
  // claro: 0,6; escuro: preto comum (0,05); furtivo: preto fosco (0,025) com laterais inclinadas
  // que desviam o infravermelho, como o chassi da proposta do Grupo D e o URC10
  M.op.p.refl = cor == "claro" ? 0.6 : (cor == "furtivo" ? 0.025 : 0.05);
  M.op.p.furtivo = cor == "furtivo";
  if (vBat < 0) vBat = M.uniforme(11.4, 12.6);
  M.nosso.p.vBat = vBat;
  int jeitoNosso = (int)M.uniforme(0, 5), jeitoOp = (int)M.uniforme(0, 5);
  posiciona(M, M.nosso, +1, jeitoNosso);
  posiciona(M, M.op, -1, jeitoOp);
  if (pessoas) {
    // pessoas e juiz em volta do dojô: pernas de 12 a 80 cm da borda, refletância de roupa comum
    int n = 4 + (int)M.uniforme(0, 3);
    for (int k = 0; k < n; k++) {
      double a = M.uniforme(-M_PI, M_PI), r = R_DOJO + M.uniforme(0.12, 0.80);
      M.fora.push_back({r * std::cos(a), r * std::sin(a), 0.06, M.uniforme(0.2, 0.6)});
    }
  }
  if (!arqRastro.empty()) { rastro = fopen(arqRastro.c_str(), "w"); if (rastro) fprintf(rastro, "{\"quadros\":[\n"); }

  IA ia; ia.tipo = tipo;
  // boot do firmware (o robô está parado no dojô, ainda sem start)
  motores_init();
  cinematica_init();
  percepcao_init();
  estados_init();

  const double dtFis = 0.00025;
  const uint64_t passoLoop = 1000;      // loop do firmware a cada 1 ms
  uint64_t tStart = H.us + 200000;      // start 0,2 s depois do boot
  bool apertou = false;
  double tLiberado = -1;                // instante em que os robôs podem andar (5 s depois do start)
  uint64_t proxRastro = 0, proxIa = 0, proxDiag = 0;
  int resultado = 0;                    // 1 vitória, -1 derrota empurrado, -2 queda própria, 0 empate
  double tFim = 0;
  int bordas = 0;
  Estado estAnt = estado_atual();
  double tFantasma = 0;                 // tempo mirando ou atacando algo fora do dojô
  // detecção antes do primeiro contato: tempo até a primeira estimativa certa e parte do tempo
  // em que o oponente estava dentro de algum cone e a estimativa estava certa (estabilidade)
  double tDet = -1;
  long nVisivel = 0, nCerto = 0;
  bool jaEncostou = false;
  M.t = H.us * 1e-6;
  while (true) {
    // botão de start: aperta por 50 ms
    if (!apertou && H.us >= tStart) { H.botao = true; if (H.us >= tStart + 50000) { H.botao = false; apertou = true; tLiberado = H.us * 1e-6 + 5.0; } }
    // firmware
    percepcao_atualiza();
    estados_passo();
    percepcao_manutencao(estado_atual() == ESPERA && !estados_teste_ativo());
    Estado e = estado_atual();
#if __has_include("arena.h")
    bool fugaAnt = estAnt == RECUO || estAnt == AVANCO_BORDA || estAnt == SAIDA_LATERAL || estAnt == GIRO_BORDA;
    bool fuga = e == RECUO || e == AVANCO_BORDA || e == SAIDA_LATERAL || e == GIRO_BORDA;
#else
    bool fugaAnt = estAnt == RECUO || estAnt == AVANCO_BORDA || estAnt == GIRO_BORDA;
    bool fuga = e == RECUO || e == AVANCO_BORDA || e == GIRO_BORDA;
#endif
    if (e != estAnt && !fugaAnt && fuga) bordas++;
    estAnt = e;
    // perseguição de fantasma: mirando ou atacando, com a estimativa mais perto de um objeto de fora
    if ((e == MIRA || e == ATAQUE) && !M.fora.empty() && P.op.visto) {
      double c = std::cos(M.nosso.th), s = std::sin(M.nosso.th);
      double ex = M.nosso.x + (c * P.op.x - s * P.op.y) / 1000, ey = M.nosso.y + (s * P.op.x + c * P.op.y) / 1000;
      double dOp = std::hypot(ex - M.op.x, ey - M.op.y), dFora = 1e9;
      for (auto& o : M.fora) dFora = std::min(dFora, std::hypot(ex - o.x, ey - o.y));
      if (dFora + 0.08 < dOp) tFantasma += passoLoop * 1e-6;
    }
    if (tLiberado > 0 && M.t >= tLiberado && !M.op.fora) {
      if (M.contato) jaEncostou = true;
      double ox, oy;
      M.nosso.paraCorpo(M.op.x, M.op.y, ox, oy);
      double angReal = std::atan2(oy, ox) * 180 / M_PI;
      double err = std::fabs(std::remainder(P.op.angulo - angReal, 360.0));
      bool certo = P.op.visto && err < 20;
      if (tDet < 0 && P.op.visto && err < 15) tDet = M.t - tLiberado;
      if (!jaEncostou) {
        bool vis = false;
        for (uint8_t i = 0; i < N_TOF && !vis; i++)
          for (int k = -1; k <= 1 && !vis; k++) {
            double d = M.alcanceVisivel(TOF[i].x, TOF[i].y, TOF[i].ang + 9.0 * k, nullptr);
            if (d > 0 && d < 520) vis = true;
          }
        if (vis) { nVisivel++; if (certo) nCerto++; }
      }
    }
    // motores do robô do Grupo D: tensão média a partir do PWM nos pinos IN
    double vDisp = M.nosso.p.vBat - M.nosso.p.quedaPonte;
    M.nosso.rodaE.v = tensaoMotorEsq(vDisp);
    M.nosso.rodaD.v = tensaoMotorDir(vDisp);
    // IA do oponente a cada 5 ms
    bool liberado = tLiberado > 0 && M.t >= tLiberado;
    if (H.us >= proxIa) { iaPasso(M, ia, 0.005, liberado); proxIa = H.us + 5000; }
    // física até o próximo loop
    uint64_t alvo = H.us + passoLoop;
    while (H.us < alvo) { M.passo(dtFis); H.us += (uint64_t)(dtFis * 1e6); }
    M.t = H.us * 1e-6;
    if (rastro && H.us >= proxRastro) { gravaRastro(M); proxRastro = H.us + 20000; }
#if __has_include("arena.h")
    if (diag && H.us >= proxDiag && liberado) {
      proxDiag = H.us + 100000;
      // centro do dojô de verdade no referencial do robô, contra a estimativa da arena
      double rx, ry, ox, oy;
      M.nosso.paraCorpo(0, 0, rx, ry);
      M.nosso.paraCorpo(M.op.x, M.op.y, ox, oy);
      printf("t=%6.2f %-13s arena %s c=(%5.0f,%5.0f) real (%5.0f,%5.0f) sig %4.0f | op %s est (%5.0f,%5.0f) real (%5.0f,%5.0f) ap %4.0f col %02x %s\n",
             M.t - tLiberado, estado_nome(estado_atual()), P.arena.confiavel ? "OK" : (P.arena.valida ? "? " : "--"),
             P.arena.cx, P.arena.cy, rx * 1000, ry * 1000, P.arena.sigma, P.op.visto ? "V" : "-", P.op.x, P.op.y,
             ox * 1000, oy * 1000, P.op.aproximacao, P.op.colados, P.op.contato ? "CONTATO" : "");
    }
#endif
    // fim do round
    if (M.nosso.fora || M.op.fora) {
      if (M.nosso.fora && M.op.fora) resultado = 0;
      else if (M.op.fora) resultado = 1;
      else resultado = (M.t - M.tUltContato < 0.3) ? -1 : -2;
      tFim = M.t - (tLiberado > 0 ? tLiberado : M.t);
      break;
    }
    if (tLiberado > 0 && M.t >= tLiberado + tRound) { resultado = 0; tFim = tRound; break; }
    if (!apertou && H.us > 3000000) break;   // start não aceito (sensores faltando)
  }
  if (rastro) {
    fprintf(rastro, "\n],\"oponente\":\"%s\",\"cor\":\"%s\",\"dip\":%d,\"semente\":%u,\"resultado\":%d,\"tempo\":%.2f,"
            "\"liberado\":%.3f,\"fora\":[", tipo.c_str(), cor.c_str(), dip, semente, resultado, tFim, tLiberado);
    for (size_t k = 0; k < M.fora.size(); k++) fprintf(rastro, "%s[%.0f,%.0f,%.0f]", k ? "," : "", M.fora[k].x * 1000, M.fora[k].y * 1000, M.fora[k].raio * 1000);
    // nomes dos estados na ordem do enum (o visualizador não guarda uma cópia da lista)
    fprintf(rastro, "],\"estados\":[");
    for (int k = 0; k < N_ESTADOS; k++) fprintf(rastro, "%s\"%s\"", k ? "," : "", estado_nome((Estado)k));
#if __has_include("chave.h")
    fprintf(rastro, "],\"estrategia\":\"%s\",\"escuro\":%s}\n", estrategia_nome(estados_estrategia()), estados_escuro() ? "true" : "false");
#else
    fprintf(rastro, "]}\n");
#endif
    fclose(rastro);
  }
  static const char* NOME[] = {"queda_propria", "derrota", "empate", "vitoria"};
  printf("%u,%s,%s,%d,%s,%.2f,%.2f,%d,%.2f,%.2f,%d,%d,%.3f,%.3f\n", semente, tipo.c_str(), cor.c_str(), dip,
         NOME[resultado + 2], tFim, M.tContatoTotal, bordas, tFantasma, vBat, jeitoNosso, jeitoOp,
         tDet, nVisivel ? (double)nCerto / nVisivel : -1.0);
  return 0;
}
