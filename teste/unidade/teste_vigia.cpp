// teste_vigia.cpp
// Testes da vigia com o dojô vazio (config.h VIGIA_*), com a máquina de estados de verdade no computador e um mundo
// mínimo em volta: o robô gira e anda conforme a cinemática estima, o oponente fica num rumo fixo no mundo (ou não
// existe), e os ToF da frente só veem o que estiver a até 32 graus do centro (a cada 30 ms, com uma leitura de atraso).
// O rastreador é imitado: "existe" vale 1 com o oponente visto e o valor escolhido pelo teste sem ele, e o rumo segue
// as mesmas regras do percepcao.cpp. Confere que a BUSCA normal (avanços curtos) continua igual nos primeiros segundos,
// que com o dojô vazio o robô para no lugar e só varre um setor com a frente (pulsos de VIGIA_PULSO_DEG: 3 para a esquerda,
// 6 para a direita, 6 para a esquerda...), sem sair dele, que sai da vigia na hora em que vê o oponente (na frente ou no
// canto do setor varrido), que não entra com o rastreador achando que existe oponente, e que a borda continua com
// prioridade.
// Uso (uma linha só, FW = firmware/sumo_esp32):
//   g++ -std=c++11 -Wall -Wextra -I teste/unidade/stub -I $FW teste/unidade/teste_vigia.cpp $FW/estados.cpp
//       $FW/cinematica.cpp $FW/arena.cpp $FW/chave.cpp $FW/antitrava.cpp -o t && ./t
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "estados.h"
#include "percepcao.h"
#include "cinematica.h"
#include "telemetria.h"
#include "arena.h"
#include "chave.h"

uint32_t relogio_us = 0;
Percepcao P;
static int16_t pwmE = 0, pwmD = 0;
void motores(int16_t esq, int16_t dir) { pwmE = esq; pwmD = dir; }
void motores_para() { motores(0, 0); }
static std::string texto;
size_t Saida::write(uint8_t c) { texto.push_back((char)c); return 1; }
size_t Saida::write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
Saida Log;
int digitalRead(int) { return 0; }
void tof_perfil(bool) {}
bool tof_perfil_escuro() { return false; }
bool oponente_borda_tras_recente() { return false; }

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor) {
  printf("%s  %-84s %9.2f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}

// ---- mundo ----
static float psi = 0, beta = 0;                 // rumo do robô e do oponente no mundo (graus, positivo é esquerda)
static bool  oponentePresente = false;
static float existeSemVer = 0;                  // o que o rastreador diria sem ver o oponente
static float andou = 0, giro = 0;               // distância percorrida (mm, em módulo) e giro (graus, em módulo)
static float ultE = 0; static int8_t ultLado = 0; static uint32_t tUltVisto = 0;
static bool veMedido = false, veAnterior = false; static float eMedido = 0, eAnterior = 0;
static float envolve(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
static void passo(int ms = 2) {
  for (int t = 0; t < ms; t += 2) {
    relogio_us += 2000; P.t_ms = millis();
    if ((P.t_ms % 30) < 2) {
      veAnterior = veMedido; eAnterior = eMedido;
      float e = envolve(beta - psi);
      veMedido = oponentePresente && fabsf(e) <= 32.0f; eMedido = e;
      P.op.visto = veAnterior; P.op.centrado = veAnterior && fabsf(eAnterior) <= CENTRO_DEG;
      P.op.mascara = veAnterior ? (uint16_t)(1u << 2) : 0;
      if (veAnterior) { P.op.angulo = eAnterior; P.op.dist_mm = 300; P.op.existe = 1; P.op.confianca = 1;
                        ultLado = eAnterior > 0 ? 1 : -1; tUltVisto = P.t_ms; ultE = eAnterior; }
      else { P.op.angulo = ultE; P.op.dist_mm = -1; P.op.existe = existeSemVer; P.op.confianca = existeSemVer; }
      P.op.lado_ultimo = ultLado;
      // rumo como no percepcao.cpp
      Rumo& r = P.rumo;
      if (P.op.visto) { r.modo = P.op.centrado ? RUMO_ATACAR : RUMO_MIRAR; r.angulo = P.op.angulo; }
      else if (P.op.existe >= 0.3f && P.op.confianca >= 0.15f) { r.modo = RUMO_PROVAVEL; r.angulo = P.op.angulo; }
      else { r.modo = RUMO_PROCURAR; r.angulo = 90.0f * (ultLado != 0 ? ultLado : 1); }
    }
    P.op.ms_sem_ver = P.op.visto ? 0 : (tUltVisto ? P.t_ms - tUltVisto : 99999);
    estados_passo(); P.start = false;
    psi += cinematica_w_dps() * 0.002f;
    andou += fabsf(cinematica_v_mms()) * 0.002f; giro += fabsf(cinematica_w_dps()) * 0.002f;
  }
}
static bool contem(const char* s) { return texto.find(s) != std::string::npos; }
static void zera() { andou = 0; giro = 0; }
// começa uma luta com o mundo escolhido; devolve com a luta já em andamento (contagem de 5 s vencida)
static void luta(bool presente, float rumoOponente, float existe) {
  estados_parar(); passo(20); texto.clear();
  psi = 0; beta = rumoOponente; oponentePresente = presente; existeSemVer = existe;
  ultE = 0; ultLado = 0; tUltVisto = 0; veMedido = veAnterior = false;
  P.op = Oponente{}; P.borda = Borda{}; P.saude.pronto = true; P.start = false;
  passo(40);
  estados_iniciar(); passo(10);
  passo(T_CONTAGEM_MS + 50);
}

int main() {
  memset(&P, 0, sizeof(P));
  cinematica_init(); estados_init(); chave_init(); arena_init();
  P.saude.pronto = true;
  passo(50);

  // 1. dojô vazio: nos primeiros segundos, a BUSCA de sempre (gira e dá avanços curtos)
  luta(false, 0, 0);
  confere(estado_atual() == BUSCA, "dojô vazio: a luta começa em BUSCA", estado_atual());
  zera();
  passo(VIGIA_SEM_VER_MS - 500);
  confere(!contem("# vigia:") && andou > 30.0f, "antes de VIGIA_SEM_VER_MS a BUSCA continua igual (avanços curtos, mm)", andou);

  // 2. depois do tempo e de uma volta inteira, a vigia: parado no lugar, só a frente varre o setor de +-45 graus
  float psiVigia = 0; bool achouInicio = false;
  for (int t = 0; t < 6000 && !achouInicio; t += 2) { passo(); if (contem("# vigia:")) { achouInicio = true; psiVigia = psi; } }
  confere(achouInicio && estado_atual() == BUSCA, "com o dojô vazio entra na vigia e avisa no terminal", estado_atual());
  float centro = psiVigia;                             // o meio do setor é o rumo em que a vigia começou
  zera();
  float mn = 1e9f, mx = -1e9f; int trocas = 0; float ultPsi = psi; int ultSinal = 0;
  std::string seq;                                     // sentido de cada pulso, na ordem
  for (int t = 0; t < 30000; t += 2) {
    passo();
    float rel = psi - centro;
    if (rel < mn) mn = rel;
    if (rel > mx) mx = rel;
    float d = psi - ultPsi;
    if (fabsf(d) > 0.05f) { int sg = d > 0 ? 1 : -1; if (sg != ultSinal) { trocas++; seq += sg > 0 ? 'E' : 'D'; ultSinal = sg; } }
    ultPsi = psi;
  }
  float lado = VIGIA_LADO_PULSOS * VIGIA_PULSO_DEG;
  confere(andou < 10.0f, "vigia por 30 s: nenhum avanço (distância estimada, mm)", andou);
  confere(mx >= lado - 8 && mx <= lado + 12, "a frente vai até uns +45 graus (esquerda) e não passa (graus)", mx);
  confere(mn <= -lado + 8 && mn >= -lado - 12, "e até uns -45 graus (direita) e não passa (graus)", mn);
  confere(seq.size() >= 4 && seq[0] == 'E', "começa para a esquerda e vai trocando de lado (E, D, E, D...)", trocas);
  confere(estado_atual() == BUSCA, "continua em BUSCA, sem lutar com o vazio", estado_atual());
  printf("       sentidos dos trechos: %s | giro total %.0f graus em 30 s\n", seq.c_str(), giro);

  // 3. na vigia, o oponente aparece na frente: sai na hora e vai lutar
  beta = psi + 10; oponentePresente = true; existeSemVer = 0;
  passo(400);
  confere(estado_atual() != BUSCA && contem("vigia: fim (oponente visto)"), "oponente aparece na frente: sai da vigia e luta", estado_atual());

  // 4. na vigia, o oponente é posto a 70 graus do rumo da vigia (fora do cone parado, dentro do setor varrido): é achado
  luta(false, 0, 0);
  psiVigia = 0; achouInicio = false;
  for (int t = 0; t < 12000 && !achouInicio; t += 2) { passo(); if (contem("# vigia:")) { achouInicio = true; psiVigia = psi; } }
  confere(achouInicio, "dojô vazio de novo: vigia", 0);
  passo(300);
  beta = psi - 70; oponentePresente = true;            // à direita
  int tAchou = -1;
  for (int t = 0; t < 30000; t += 2) { passo(); if (estado_atual() != BUSCA) { tAchou = t; break; } }
  confere(tAchou >= 0 && tAchou <= 10 * (VIGIA_PAUSA_MS + 250), "oponente a 70 graus para o lado: a varredura o encontra (ms)", tAchou);
  // e o que fica atrás não faz o robô girar até lá: ele continua no setor da frente
  luta(false, 0, 0);
  achouInicio = false;
  for (int t = 0; t < 12000 && !achouInicio; t += 2) { passo(); if (contem("# vigia:")) { achouInicio = true; centro = psi; } }
  beta = psi + 180; oponentePresente = true;
  mn = 1e9f; mx = -1e9f;
  for (int t = 0; t < 20000; t += 2) { passo(); float rel = psi - centro; if (rel < mn) mn = rel; if (rel > mx) mx = rel; }
  confere(estado_atual() == BUSCA && mx <= lado + 12 && mn >= -lado - 12, "oponente atrás: o robô não sai do setor da frente (graus para a esquerda)", mx);

  // 5. o rastreador acha que existe oponente (rumo provável): nada de vigia, a BUSCA segue procurando
  luta(false, 0, 0.6f);
  zera();
  passo(VIGIA_SEM_VER_MS + 8000);
  confere(!contem("# vigia:"), "rastreador com o oponente provável: não entra na vigia", 0);

  // 6. a borda tem prioridade na vigia, e depois da fuga a vigia volta
  luta(false, 0, 0);
  passo(VIGIA_SEM_VER_MS + 6000);
  texto.clear();
  P.borda.detectada = true; P.borda.mascara = 1; P.borda.cru = 1; P.borda.frente = true; P.borda.lado = -1;
  passo(60);
  confere(estado_atual() == RECUO || estado_atual() == GIRO_BORDA || estado_atual() == SAIDA_LATERAL,
          "borda na vigia: a fuga tem prioridade", estado_atual());
  P.borda = Borda{};
  passo(8000);
  confere(estado_atual() == BUSCA && contem("# vigia:"), "depois da fuga, de volta à BUSCA e à vigia", estado_atual());

  estados_parar();
  printf(falhas ? "\nERRO: %d falha(s)\n" : "\nOK: 0 falha(s)\n", falhas);
  return falhas ? 1 : 0;
}
