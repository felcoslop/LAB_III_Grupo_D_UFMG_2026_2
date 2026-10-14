// teste_estados.cpp
// Testes da máquina de estados junto com a cinemática, no computador. A percepção P é montada à
// mão em cada passo, como se fosse a foto dos sensores, e o teste confere o estado e o PWM que
// chega nos motores. O foco é a borda: ela precisa ganhar de qualquer outro estado na hora. Os
// testes do fim conferem as estratégias da chave DIP (Rush, Flank e Fan out), a fuga lateral, o
// ataque de ré e o contato segurado.
// Uso (uma linha só):
//   g++ -std=c++11 -Wall -Wextra -I teste/stub -I . teste/teste_estados.cpp estados.cpp cinematica.cpp arena.cpp chave.cpp -o t
//   ./t
#include <cstdio>
#include <cstdlib>
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
static bool mostraLog = false;
size_t Saida::write(uint8_t c) { if (mostraLog) putchar(c); return 1; }
size_t Saida::write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
Saida Log;

// chave DIP de mentira (bit 0 = DIP 1 ... bit 3 = DIP 4) e perfil dos sensores aplicado na hora
static uint8_t dip = 0;
int digitalRead(int pino) {
  const int pinos[4] = {PINO_DIP1, PINO_DIP2, PINO_DIP3, PINO_DIP4};
  for (int i = 0; i < 4; i++) if (pino == pinos[i]) return (dip >> i) & 1;
  return 1;
}
static bool perfilEscuro = false;
void tof_perfil(bool escuro) { perfilEscuro = escuro; }
bool tof_perfil_escuro() { return perfilEscuro; }
static bool trasRecente = false;
bool oponente_borda_tras_recente() { return trasRecente; }

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor) {
  printf("%s  %-66s %9.1f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}
// uma volta do loop a cada 2 ms, com a foto atual em P
static void passo(int ms = 2) {
  for (int t = 0; t < ms; t += 2) { relogio_us += 2000; P.t_ms = millis(); chave_atualiza(); estados_passo(); P.start = false; }
}
static void semBorda() { P.borda = Borda{}; }
// sensores de borda no branco, com a máscara montada a partir da tabela BORDA do config.h
static void borda(bool fe, bool fd, bool te, bool td) {
  P.borda = Borda{};
  for (uint8_t i = 0; i < N_BORDA; i++) {
    bool b = BORDA[i].frente ? (BORDA[i].lado < 0 ? fe : fd) : (BORDA[i].lado < 0 ? te : td);
    if (b) P.borda.mascara |= (1u << i);
  }
  P.borda.cru = P.borda.mascara;
  P.borda.detectada = P.borda.mascara != 0;
  P.borda.frente = fe || fd; P.borda.tras = te || td;
  bool esq = fe || te, dir = fd || td;
  P.borda.lado = (esq && dir) ? 0 : esq ? -1 : dir ? 1 : 0;
}
static void bordaFrente(int8_t lado) { borda(lado <= 0, lado >= 0, false, false); }
static void bordaTras(int8_t lado)   { borda(false, false, lado <= 0, lado >= 0); }
// oponente visto no ângulo ang, com a face a dist mm do sensor (centro a dist + 76 mm)
static void oponente(bool visto, float ang, float dist = 300) {
  P.op.visto = visto; P.op.angulo = ang; P.op.centrado = visto && fabsf(ang) <= CENTRO_DEG;
  P.op.dist_mm = visto ? (int16_t)dist : -1;
  P.op.existe = visto ? 1.0f : 0.0f;
  P.op.aproximacao = 0; P.op.contato = false;
  P.rumo.modo = visto ? (P.op.centrado ? RUMO_ATACAR : RUMO_MIRAR) : RUMO_PROCURAR;
  P.rumo.angulo = visto ? ang : 90;
  P.rumo.dist_mm = visto ? dist + 76 : 0;
}
// começa a luta: start, contagem de 5 s e entrada na BUSCA
static void luta() {
  estados_parar(); semBorda(); oponente(false, 0);
  P.arena = Arena{};
  passo(40);                                     // a chave DIP passa pelo filtro de 30 ms
  P.saude.pronto = true; P.start = true; passo(); passo(T_CONTAGEM_MS + 10);
  P.arena = Arena{};                             // sem localização confiável, a não ser que o teste diga
}

int main() {
  memset(&P, 0, sizeof(P));
  cinematica_init(); estados_init(); chave_init(); arena_init();

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
  // o comando para 10 graus antes do alvo, porque a roda ainda gira um pouco no atraso do motor
  confere(fabsf(cinematica_giro()) >= ANG_GIRO_BORDA - 11 && fabsf(cinematica_giro()) < 180, "giro de recuperacao com cerca de 135 graus estimados", fabsf(cinematica_giro()));
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

  // comando pwm: PWM direto em cada roda, sem cinemática, e parada no fim do tempo
  estados_parar(); semBorda(); passo(100);
  estados_teste_pwm(40, -100, 300); passo(100);
  confere(pwmE == 102 && pwmD == -255, "pwm 40 -100 300: PWM 102 na esquerda e -255 na direita", pwmE);
  confere(estados_teste_ativo(), "pwm: teste ativo durante o tempo pedido", estados_teste_ativo());
  passo(250);
  confere(pwmE == 0 && pwmD == 0 && !estados_teste_ativo(), "pwm: para sozinho no fim do tempo", pwmE);

  // ===================== estratégias, fugas novas e contato =====================
  // borda em um lado inteiro (frente e trás na esquerda): curva para dentro, roda direita parada
  luta();
  borda(true, false, true, false); passo();
  confere(estado_atual() == SAIDA_LATERAL, "borda na lateral esquerda: SAIDA_LATERAL", estado_atual());
  confere(pwmE == 255 && abs(pwmD) <= 5, "SAIDA_LATERAL: roda esquerda no maximo e direita parada", pwmD);
  semBorda(); passo(20);
  confere(estado_atual() == GIRO_BORDA || estado_atual() == BUSCA, "lateral livre: segue para o giro de recuperacao", estado_atual());

  // borda de trás nunca vira giro para trás: AVANCO_BORDA e depois BUSCA
  luta(); bordaTras(0); passo(4);
  confere(estado_atual() == AVANCO_BORDA, "borda atras: AVANCO_BORDA (nunca giro para tras)", estado_atual());
  semBorda(); passo(20);
  confere(estado_atual() == BUSCA, "fim do AVANCO_BORDA: BUSCA, ja de frente para dentro", estado_atual());

  // giro da borda para o centro do dojô quando a localização é confiável
  luta(); oponente(false, 0); bordaFrente(0); passo(4);
  P.arena = Arena{}; P.arena.valida = P.arena.confiavel = true; P.arena.ang = 100; P.arena.dist = 260; P.arena.sigma = 20;
  semBorda(); passo(14);
  confere(estado_atual() == GIRO_BORDA, "recuo terminado: GIRO_BORDA", estado_atual());
  passo(60);
  confere(pwmD > 0 && pwmE < 0, "GIRO_BORDA com o centro a +100 graus: gira para a esquerda", pwmD);

  // volta ao centro na busca, com a localização confiável e o robô longe do meio
  luta(); P.arena = Arena{}; P.arena.valida = P.arena.confiavel = true; P.arena.ang = 0; P.arena.dist = 280; P.arena.sigma = 20;
  passo(300);
  confere(estado_atual() == BUSCA && pwmE > 50 && pwmD > 50, "BUSCA longe do centro: anda para o centro", pwmE);

  // contato segurado: o ToF perde o alvo colado na frente, e o empurrão continua
  luta(); oponente(true, 0, 30); passo(100);
  confere(estado_atual() == ATAQUE, "oponente colado e centrado: ATAQUE", estado_atual());
  oponente(false, 0); P.op.contato = true; passo(200);
  confere(estado_atual() == ATAQUE && pwmE > 200, "sem leitura mas em contato: o ATAQUE continua", pwmE);
  P.op.contato = false; passo(4);
  confere(estado_atual() == BUSCA, "contato acabou e nada visto: BUSCA", estado_atual());

  // ataque de ré com o oponente colado atrás (7 sensores)
  luta(); oponente(true, 176, 100); passo(100);
  confere(estado_atual() == ATAQUE_RE, "oponente colado atras: ATAQUE_RE", estado_atual());
  confere(pwmE < -200 && pwmD < -200, "ATAQUE_RE: as duas rodas para tras", pwmE);
  oponente(true, 120, 300); passo(10);
  confere(estado_atual() == MIRA, "oponente saiu de tras: volta a MIRA", estado_atual());

  // giro preso: oponente encostado na lateral a +60 graus, a mira gira e o ângulo não muda
  luta(); oponente(true, 60, 10); passo(T_GIRO_TRAVA_MS + 100);
  confere(estado_atual() == DESTRAVA, "giro preso pelo oponente na lateral: DESTRAVA", estado_atual());
  confere(estados_travado() == 2 && estados_destravas() == 1, "giro preso: travamento 2 e uma manobra contada", estados_destravas());
  passo(20);
  confere(pwmE < -200 && pwmD < -200, "DESTRAVA com espaco atras: re reta", pwmE);
  passo(T_DESTRAVA_RE_MS);
  confere(estado_atual() == MIRA, "fim da DESTRAVA: volta a mirar", estado_atual());
  oponente(true, 20, 300); passo(200);
  confere(estado_atual() != DESTRAVA, "oponente mudando de angulo: nada de DESTRAVA", estado_atual());
  // sem espaço atrás (borda de trás vista há pouco): avanço reto
  luta(); oponente(true, 60, 10); trasRecente = true; passo(T_GIRO_TRAVA_MS + 100);
  confere(estado_atual() == DESTRAVA, "giro preso perto da borda de tras: DESTRAVA", estado_atual());
  passo(20);
  confere(pwmE > 200 && pwmD > 200, "DESTRAVA sem espaco atras: avanco reto", pwmE);
  trasRecente = false;
  // empurrão em contato longo: o robô continua empurrando, só o travamento 1 aparece
  luta(); oponente(true, 0, 30); passo(100);
  oponente(false, 0); P.op.contato = true; passo(T_TRAVA_MS + 100);
  confere(estado_atual() == ATAQUE && pwmE > 200 && estados_travado() == 1, "empurrao em contato longo: continua no ATAQUE (travamento 1)", pwmE);
  P.op.contato = false; passo(4);

  // Flank (DIP 2): oponente longe e na frente vira FLANCO, com o rumo desviado para um lado
  dip = 0x02; luta();
  confere(estados_estrategia() == FLANK, "DIP 2 ligada: estrategia FLANK travada no start", estados_estrategia());
  oponente(true, 10, 400); passo(60);
  confere(estado_atual() == FLANCO, "FLANK com oponente a 400 mm: FLANCO", estado_atual());
  confere(pwmE > pwmD, "FLANCO com oponente a +10 graus: curva para a direita (passa pelo lado dele)", pwmE - pwmD);
  oponente(true, 10, 150); passo(10);
  confere(estado_atual() == MIRA || estado_atual() == ATAQUE, "FLANCO perto do oponente: fecha o ataque", estado_atual());
  // investida: oponente vindo rápido de frente não recebe flanco, recebe o torque de frente
  luta(); oponente(true, 5, 400); P.op.aproximacao = 600; passo(40);
  confere(estado_atual() == ATAQUE || estado_atual() == MIRA, "FLANK com investida rapida: recebe de frente", estado_atual());

  // Fan out (DIP 3): sem pista, a busca oscila o rumo de um lado para o outro e avança devagar
  dip = 0x04; luta();
  confere(estados_estrategia() == FAN_OUT, "DIP 3 ligada: estrategia FAN_OUT", estados_estrategia());
  oponente(false, 0); passo(100);
  int sinal1 = pwmD > pwmE ? 1 : -1;
  bool trocou = false, avancou = false;
  for (int k = 0; k < 400; k++) {
    passo(2);
    if ((pwmD > pwmE ? 1 : -1) != sinal1) trocou = true;
    if (pwmE + pwmD > 20) avancou = true;
  }
  confere(trocou, "FAN_OUT: o sentido do giro troca (leque de um lado para o outro)", trocou);
  confere(avancou, "FAN_OUT: o robo avanca devagar durante o leque", avancou);

  // DIP 4: perfil escuro pedido aos sensores e travado no start; duas estratégias ligadas valem a de menor número
  dip = 0x08 | 0x06; luta();
  confere(estados_escuro() && perfilEscuro, "DIP 4 ligada: perfil de oponente escuro", estados_escuro());
  confere(estados_estrategia() == FLANK, "DIP 2 e 3 ligadas juntas: vale a DIP 2 (FLANK)", estados_estrategia());
  dip = 0; passo(100);
  confere(estados_estrategia() == FLANK && estados_escuro(), "chave mexida durante a luta: nada muda", estados_estrategia());
  dip = 0;

  printf("\n%s: %d falha(s)\n", falhas ? "ERRO" : "OK", falhas);
  return falhas ? 1 : 0;
}
