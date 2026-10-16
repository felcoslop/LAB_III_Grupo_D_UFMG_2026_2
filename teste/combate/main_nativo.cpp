// main_nativo.cpp
// Linha de comando do simulador de combate, para a bateria de lutas e para o CI.
//
// Uso: combate [opções]
//   -s N        semente (posição inicial, ruído dos sensores, estratégia sorteada)
//   -a E        estratégia do Grupo D: rush, flank ou fanout
//   -b E        estratégia do adversário: rush, flank, fanout ou sorteada (padrão)
//   -c COR      cor do adversário: preto (padrão) ou branco
//   -C COR      cor do robô do Grupo D: preto (padrão) ou branco
//   -m RPM      motores do adversário: 1000 (padrão) ou 319
//   -p N        rumo do Grupo D na largada: 0 de frente para o adversário, 1 e 2 de lado, 3 de costas
//               (padrão sorteado)
//   -P N        o mesmo para o adversário
//   -u F        fator no atrito do pneu (padrão 1: 1,15 parado e 1,30 escorregando; 0,8 com pó)
//   -e E        coeficiente de restituição do choque entre os robôs (padrão 0,4)
//   -x N        manobra do empurrão travado do Grupo D (antitrava.h; padrão a do config.h)
//   -X N        manobra do adversário (padrão 0, só detecta)
//   -t S        segundos de luta depois dos 5 s parados (padrão 90)
//   -r ARQ      grava o rastro da luta em JSON (um quadro a cada 20 ms)
//   -v          mostra o Log dos dois firmwares
// Saída: uma linha CSV com o resultado (cabeçalho com -H).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "combate.h"
#include "hal_combate.h"

using namespace cb;

static int estrategia(const char* s) {
  if (!strcmp(s, "rush")) return 0;
  if (!strcmp(s, "flank")) return 1;
  if (!strcmp(s, "fanout")) return 2;
  return -1;
}

static const char* CAB = "semente,estr_a,estr_b,cor_b,motor_b,vencedor,motivo,tempo,contato,levantado_a,levantado_b,"
                         "aquec_a,aquec_b,destravas_a,destravas_b,prim_contato,f_max,largada_a,largada_b,manobras_a,manobras_b";

int main(int argc, char** argv) {
  Config c;
  std::string rastro;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto prox = [&]() { return i + 1 < argc ? argv[++i] : (char*)""; };
    if (a == "-s") c.semente = (uint32_t)atol(prox());
    else if (a == "-a") c.estrA = std::max(0, estrategia(prox()));
    else if (a == "-b") c.estrB = estrategia(prox());
    else if (a == "-c") c.brancoB = !strcmp(prox(), "branco");
    else if (a == "-C") c.brancoA = !strcmp(prox(), "branco");
    else if (a == "-m") c.b1000 = atoi(prox()) != 319;
    else if (a == "-p") c.posA = atoi(prox());
    else if (a == "-P") c.posB = atoi(prox());
    else if (a == "-u") c.atrito = atof(prox());
    else if (a == "-e") c.restit = atof(prox());
    else if (a == "-x") c.manobraA = atoi(prox());
    else if (a == "-X") c.manobraB = atoi(prox());
    else if (a == "-t") c.tRound = atof(prox());
    else if (a == "-r") rastro = prox();
    else if (a == "-v") verboso = true;
    else if (a == "-H") { puts(CAB); return 0; }
  }
  inicia(c);
  FILE* f = rastro.empty() ? nullptr : fopen(rastro.c_str(), "w");
  if (f) fprintf(f, "{\"quadros\":[\n");
  bool primeiro = true;
  int estB = -1;
  while (true) {
    bool segue = passo(20);
    const Mundo& M = mundoAtual();
    if (f) {
      fprintf(f, "%s[%.3f", primeiro ? "" : ",\n", M.t);
      for (int k = 0; k < 2; k++) {
        const Robo& rb = M.r[k];
        double ex, ey;
        rb.eixo(ex, ey);
        FwLeitura l; leFirmware(k, l);
        fprintf(f, ",%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.3f,%d,%d,%d", ex, ey, rb.th, rb.angE, rb.angD, rb.arfagem, rb.rolagem,
                rb.fora ? 1 : 0, l.estado, l.opVisto ? 1 : 0);
      }
      fprintf(f, "]");
      primeiro = false;
    }
    if (!segue) break;
  }
  FwLeitura lb; leFirmware(1, lb); estB = lb.estrategia;
  const Resultado& R = resultado();
  const Estatistica& E = estatistica();
  if (f) {
    fprintf(f, "\n],\"vencedor\":%d,\"motivo\":\"%s\",\"tempo\":%.2f,\"liberado\":%.3f,\"estados\":[", R.vencedor,
            nomeMotivo(R.motivo), R.tempo, tLiberado());
    for (int k = 0; k < api(0).nEstados(); k++) fprintf(f, "%s\"%s\"", k ? "," : "", api(0).nomeEstado(k));
    fprintf(f, "]}\n");
    fclose(f);
  }
  static const char* EST[] = {"rush", "flank", "fanout"};
  printf("%u,%s,%s,%s,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d,%.2f,%.1f,%d,%d,%d,%d\n", c.semente, EST[c.estrA],
         EST[estB < 0 ? 0 : estB], c.brancoB ? "branco" : "preto", c.b1000 ? 1000 : 319, R.vencedor, R.motivo, R.tempo,
         E.tContato, E.tLevantado[0], E.tLevantado[1], E.aquecMax[0], E.aquecMax[1], E.destravas[0], E.destravas[1],
         E.tPrimContato, E.fMaxContato, E.jeito[0], E.jeito[1], E.manobras[0], E.manobras[1]);
  return 0;
}
