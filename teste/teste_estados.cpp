// teste_estados.cpp
// Testes da máquina de estados junto com a cinemática, no computador. A percepção P é montada à
// mão em cada passo, como se fosse a foto dos sensores, e o teste confere o estado e o PWM que
// chega nos motores. O foco é a borda: ela precisa ganhar de qualquer outro estado na hora.
// Uso: g++ -std=c++11 -Wall -Wextra -I teste/stub -I . teste/teste_estados.cpp estados.cpp cinematica.cpp -o t && ./t
#include <cstdio>
#include <cstdlib>
#include "estados.h"
#include "percepcao.h"
#include "cinematica.h"
#include "telemetria.h"

uint32_t relogio_us = 0;
Percepcao P;
static int16_t pwmE = 0, pwmD = 0;
void motores(int16_t esq, int16_t dir) { pwmE = esq; pwmD = dir; }
void motores_para() { motores(0, 0); }
static bool mostraLog = false;
size_t Saida::write(uint8_t c) { if (mostraLog) putchar(c); return 1; }
size_t Saida::write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
Saida Log;

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor) {
  printf("%s  %-66s %9.1f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}
// uma volta do loop a cada 2 ms, com a foto atual em P
static void passo(int ms = 2) {
  for (int t = 0; t < ms; t += 2) { relogio_us += 2000; P.t_ms = millis(); estados_passo(); P.start = false; }
}
static void semBorda() { P.borda = Borda{}; }
static void bordaFrente(int8_t lado) { P.borda = Borda{}; P.borda.detectada = true; P.borda.frente = true; P.borda.lado = lado; }
static void bordaTras(int8_t lado)   { P.borda = Borda{}; P.borda.detectada = true; P.borda.tras = true; P.borda.lado = lado; }
static void oponente(bool visto, float ang) {
  P.op.visto = visto; P.op.angulo = ang; P.op.centrado = visto && fabsf(ang) <= CENTRO_DEG;
  P.rumo.modo = visto ? (P.op.centrado ? RUMO_ATACAR : RUMO_MIRAR) : RUMO_PROCURAR;
  P.rumo.angulo = visto ? ang : 90;
}
// começa a luta: start, contagem de 5 s e entrada na BUSCA
static void luta() {
  estados_parar(); semBorda(); oponente(false, 0);
  P.saude.pronto = true; P.start = true; passo(); passo(T_CONTAGEM_MS + 10);
}

int main() {
  memset(&P, 0, sizeof(P));
  cinematica_init(); estados_init();

  luta();
  confere(estado_atual() == BUSCA, "start + 5 s: entra na BUSCA", estado_atual());
  passo(300);
  confere(pwmE < 0 && pwmD > 0, "BUSCA sem pista: gira no eixo para a esquerda (rumo +90)", pwmD);

  // MIRA: oponente a 30 graus na esquerda faz curva para a esquerda (roda direita mais rápida)
  oponente(true, 30); passo(300);
  confere(estado_atual() == MIRA, "oponente a 30 graus: MIRA", estado_atual());
  confere(pwmD > pwmE && pwmD > 0, "MIRA a +30: roda direita mais rapida (curva para a esquerda)", pwmD - pwmE);
  confere(cinematica_v_mms() > 50, "MIRA a +30: o robo tambem avanca (curva, nao giro no eixo)", cinematica_v_mms());
  // oponente a 70 graus na direita: giro no eixo para a direita
  oponente(true, -70); passo(300);
  confere(pwmE > 0 && pwmD < 0 && fabsf(cinematica_v_mms()) < 20, "MIRA a -70: giro no eixo para a direita", cinematica_w_dps());

  // ATAQUE: centrado, velocidade máxima
  oponente(true, 2); passo(400);
  confere(estado_atual() == ATAQUE, "oponente centrado: ATAQUE", estado_atual());
  confere(pwmE > 230 && pwmD > 230, "ATAQUE: as duas rodas perto do maximo", pwmE);

  // borda na frente em pleno ataque: RECUO com tensão toda para trás na mesma volta
  bordaFrente(-1); passo();
  confere(estado_atual() == RECUO, "ATAQUE + borda na frente: RECUO na mesma volta", estado_atual());
  confere(pwmE == -255 && pwmD == -255, "RECUO: PWM -255 nas duas rodas ja na primeira volta", pwmE);
  // a borda continua enquanto houver branco, e o oponente centrado não tira o robô do RECUO
  passo(100);
  confere(estado_atual() == RECUO && pwmE == -255, "RECUO continua enquanto a frente ve branco (mesmo com oponente)", pwmE);
  // o branco some: mais 10 ms e GIRO_BORDA para a direita (borda estava na esquerda)
  semBorda(); oponente(false, 0); passo(8);
  confere(estado_atual() == RECUO, "branco sumiu: RECUO ainda segue a folga de 10 ms", estado_atual());
  passo(6);
  confere(estado_atual() == GIRO_BORDA, "folga cumprida: GIRO_BORDA", estado_atual());
  passo(100);
  confere(pwmE > 0 && pwmD < 0, "GIRO_BORDA com a borda na esquerda: gira para a direita", pwmE);
  // borda volta na frente no meio do giro: RECUO de novo (a borda é vigiada no giro)
  bordaFrente(+1); passo();
  confere(estado_atual() == RECUO, "borda de novo no meio do GIRO_BORDA: volta ao RECUO", estado_atual());
  semBorda(); passo(20);
  // giro termina pelo ângulo estimado (135 graus), e não por tempo fixo
  uint32_t t0 = millis();
  while (estado_atual() == GIRO_BORDA && millis() - t0 < 2000) passo();
  confere(estado_atual() == BUSCA, "GIRO_BORDA termina e volta para a BUSCA", estado_atual());
  confere(fabsf(cinematica_giro()) >= 134 && fabsf(cinematica_giro()) < 180, "giro de recuperacao com cerca de 135 graus estimados", fabsf(cinematica_giro()));
  confere(millis() - t0 < T_GIRO_BORDA_MAX_MS, "giro acaba pelo angulo, antes do tempo de seguranca", millis() - t0);

  // empurrão por trás: AVANCO_BORDA com tensão toda para a frente
  bordaTras(0); passo();
  confere(estado_atual() == AVANCO_BORDA && pwmE == 255 && pwmD == 255, "borda atras: AVANCO_BORDA com PWM 255 na hora", pwmE);
  semBorda(); passo(20);
  confere(estado_atual() == BUSCA, "branco atras sumiu: volta para a BUSCA", estado_atual());

  // frente e trás juntas (robô de lado na linha): GIRO_BORDA direto e fica nele
  bordaFrente(0); P.borda.tras = true; passo(10);
  confere(estado_atual() == GIRO_BORDA, "borda na frente e atras: GIRO_BORDA", estado_atual());

  // comando mov em ESPERA: anda e para sozinho; borda interrompe na hora
  estados_parar(); semBorda(); passo(400);
  estados_teste_mov(300, 0, 500); passo(250);
  confere(pwmE > 0 && pwmD > 0, "mov 300 0 500: as duas rodas para a frente", pwmE);
  passo(300);
  confere(pwmE == 0 && pwmD == 0, "mov: para sozinho no fim do tempo", pwmE);
  estados_teste_mov(300, 0, 2000); passo(100); bordaFrente(0); passo();
  confere(pwmE == 0 && pwmD == 0, "mov: borda vista para o teste na hora", pwmE);

  printf("\n%s: %d falha(s)\n", falhas ? "ERRO" : "OK", falhas);
  return falhas ? 1 : 0;
}
