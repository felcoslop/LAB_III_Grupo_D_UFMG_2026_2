// estados.cpp
// Implementação da máquina de estados, com as regras do Blackbook [DEDE, s.d.]:
//   capítulo 2: o robô só ataca quando o sensor central vê o alvo centrado; nos outros casos ele mira;
//   capítulo 10: a decisão usa onde o oponente foi visto por último, o recuo de borda dura
//   enquanto o sensor vê branco e mais uma folga de 10 ms, o giro de recuperação continua
//   vigiando o oponente e o modo defensivo nunca avança;
//   capítulo 9: a borda chega aqui já confirmada pelo filtro de tempo do borda.cpp.
// As três estratégias da proposta do Grupo D (Rush, Flank e Fan out) saem da chave DIP e ficam
// travadas no start. Elas mudam a busca e o jeito de chegar no oponente; a borda vem antes de
// tudo nas três, e a fuga da borda termina virada para o centro do dojô (arena.cpp).
// A função estados_passo() é chamada pelo loop() do sumo_esp32.ino logo depois da percepção.
// Cada estado pede um avanço em mm/s e um giro em graus/s; a cinemática (cinematica.cpp)
// transforma o pedido no PWM de cada roda. A fuga da borda usa o perfil PERFIL_BORDA, que não
// tem rampa e usa a tensão toda na hora.

#include "estados.h"
#include "config.h"
#include "telemetria.h"
#include "percepcao.h"
#include "cinematica.h"
#include "motores.h"
#include "arena.h"
#include "chave.h"
#include "oponente.h"

static Estado   est = ESPERA;
static uint32_t tEntrada = 0;          // momento em que o estado atual começou
static uint32_t tBordaLimpa = 0;       // última vez em que a borda ainda estava sendo vista
static int8_t   ladoBorda = 0;         // lado da borda que disparou o desvio (-1, 0 ou +1)
static float    alvoGiro = 0;          // tamanho do GIRO_BORDA em graus (positivo é esquerda)
static bool     pedidoStart = false;   // start pedido pelo comando "go" (debug ou painel)
static bool     buscaAvanca = false;   // na BUSCA, true indica o avanço curto que muda o ponto de vista
static uint32_t tBusca = 0;            // início do giro ou do avanço atual da BUSCA
static Estrategia estrategia = RUSH;   // travada no start
static bool     escuro = false;        // perfil do oponente, travado no start
static int8_t   ladoFlanco = 1;        // +1 passa pela esquerda do oponente, -1 pela direita
static uint32_t tFimFlanco = 0;        // fim do último flanco (o flanco só volta depois de um tempo)
static bool     aberturaFlanco = false; // FLANCO da abertura do Flank: arco largo, sem esperar a investida acabar
static uint32_t tFimLeque = 0;         // fim do último zigue-zague do Fan out
static float    fanGiro = 0;           // giro acumulado no leque (graus)
static int8_t   fanSentido = 1;
static uint32_t tAntPasso = 0;
static uint32_t tReVisto = 0;          // última vez com o oponente colado atrás
static uint32_t tEmpurra = 0;          // começo do empurrão em contato atual (0 sem empurrão)
static bool     avisouEmpurrao = false;
static uint32_t tGiroRef = 0;          // janela da conferência do giro preso (0 fora da mira)
static float    angGiroRef = 0, giroRef = 0;
static uint16_t destravas = 0;         // manobras DESTRAVA nesta luta
static int8_t   ladoDestrava = 1;      // lado do oponente na manobra (+1 esquerda)
static bool     destravaComRe = true;  // ré com espaço atrás; sem espaço, avanço reto
static uint32_t tReBateu = 0;          // última DESTRAVA de ré que terminou na borda de trás
// movimento de teste do comando "mov", só em ESPERA (Blackbook, capítulo 11: medir as primitivas)
static bool     movAtivo = false;
static bool     movBruto = false;      // true no teste pwm: PWM direto, sem passar pela cinemática
static float    movV = 0, movW = 0;
static int16_t  movE = 0, movD = 0;
static uint32_t movFim = 0, movIni = 0;

static const char* NOMES[N_ESTADOS] = {"ESPERA", "CONTAGEM", "BUSCA", "MIRA", "ATAQUE", "FLANCO", "ATAQUE_RE",
                                       "DESTRAVA", "RECUO", "GIRO_BORDA", "AVANCO_BORDA", "SAIDA_LATERAL", "LEQUE"};

const char* estado_nome(Estado e) { return e < N_ESTADOS ? NOMES[e] : "?"; }
Estado      estado_atual()        { return est; }
Estrategia  estados_estrategia()  { return estrategia; }
bool        estados_escuro()      { return escuro; }
uint16_t    estados_destravas()   { return destravas; }

// Troca de estado. Toda troca aparece no log e reinicia os tempos usados pelos estados.
static void entra(Estado novo) {
  if (novo == est) return;
  Log.printf("# estado: %s -> %s\n", NOMES[est], NOMES[novo]);
  if (est == FLANCO) { tFimFlanco = P.t_ms; aberturaFlanco = false; }
  if (est == LEQUE) tFimLeque = P.t_ms;
  est = novo;
  tEntrada = P.t_ms;
  if (novo == BUSCA) { buscaAvanca = false; tBusca = P.t_ms; fanGiro = 0; }
  if (novo == GIRO_BORDA) cinematica_zera_odometria();   // o giro de recuperação é medido a partir daqui
  if (novo == ATAQUE_RE) tReVisto = P.t_ms;
  tBordaLimpa = P.t_ms;
}

// Os estados de luta são todos os que vêm depois da CONTAGEM na ordem do enum.
static bool emLuta() { return est >= BUSCA; }

// LED de status. Com PINO_LED -1 (pino ocupado pela chave DIP) a função não faz nada.
static void led(bool aceso) {
#if PINO_LED >= 0
  digitalWrite(PINO_LED, aceso ? HIGH : LOW);
#else
  (void)aceso;
#endif
}

void estados_init() {
#if PINO_LED >= 0
  pinMode(PINO_LED, OUTPUT);
#endif
  est = ESPERA; tEntrada = millis(); tAntPasso = millis();
}

uint8_t estados_travado() {
  if (est == DESTRAVA) return 2;
  return tEmpurra && P.t_ms - tEmpurra >= T_TRAVA_MS ? 1 : 0;
}
void estados_iniciar() { pedidoStart = true; }
void estados_parar()   { entra(ESPERA); movAtivo = false; cinematica_para(); }

// Movimento de teste: o robô parado em ESPERA anda com avanço v e giro w por ms milissegundos e
// no fim o terminal mostra o que a cinemática estimou. A distância medida com a régua e o giro
// medido no chão, comparados com a estimativa, acertam o modelo (comando cin e config.h).
// Qualquer borda vista para o teste na hora.
void estados_teste_mov(float v_mms, float w_dps, uint16_t ms) {
  if (est != ESPERA) { Log.println(F("! mov so funciona com o robo parado (ESPERA)")); return; }
  movV = v_mms; movW = w_dps; movIni = millis(); movFim = movIni + (ms > 3000 ? 3000 : ms);
  cinematica_zera_odometria();
  movBruto = false;
  movAtivo = true;
  Log.printf("# mov: %.0f mm/s e %.0f graus/s por %u ms\n", movV, movW, (unsigned)(movFim - movIni));
}

// Teste de PWM direto, usado para conferir os fios: cada roda recebe a porcentagem pedida
// (-100 a 100, positivo é para a frente) por ms milissegundos, sem cinemática nem rampa.
void estados_teste_pwm(int8_t esq_pct, int8_t dir_pct, uint16_t ms) {
  if (est != ESPERA) { Log.println(F("! pwm so funciona com o robo parado (ESPERA)")); return; }
  esq_pct = constrain(esq_pct, -100, 100); dir_pct = constrain(dir_pct, -100, 100);
  movE = esq_pct * 255 / 100; movD = dir_pct * 255 / 100;
  movIni = millis(); movFim = movIni + (ms > 3000 ? 3000 : ms);
  movBruto = true;
  movAtivo = true;
  Log.printf("# pwm: esquerda %d%% (%d) e direita %d%% (%d) por %u ms\n", esq_pct, movE, dir_pct, movD,
             (unsigned)(movFim - movIni));
}

bool estados_teste_ativo() { return movAtivo; }

// Mira no oponente. O giro é proporcional ao ângulo (K_MIRA graus/s por grau), limitado a
// GIRO_MIRA_MAX_DPS. Até ANG_MIRA_CURVA o robô também avança, com v = VEL_MAX.cos(ângulo), e o
// caminho vira uma curva para o oponente; mais aberto que isso, o robô só gira no eixo. Quando
// as duas coisas não cabem na velocidade das rodas, a cinemática corta o avanço e mantém o giro.
// No modo defensivo o robô nunca avança.
static void mira(float ang) {
  float w = constrain(K_MIRA * ang, -(float)GIRO_MIRA_MAX_DPS, (float)GIRO_MIRA_MAX_DPS);
#if MODO_DEFENSIVO
  cinematica_comanda(0, w);
#else
  float v = fabsf(ang) <= ANG_MIRA_CURVA ? VEL_MAX_MMS * cosf(ang * (float)DEG_TO_RAD) : 0;
  cinematica_comanda(v, w);
#endif
}

// Investida: o oponente vem rápido de frente. Nas três estratégias o robô recebe de frente e
// empurra com o torque todo, em vez de oferecer a lateral numa curva.
static bool investida() {
  return P.op.visto && P.op.aproximacao > APROX_RAPIDA_MMS && fabsf(P.op.angulo) <= APROX_RAPIDA_DEG;
}

// O flanco só vale na estratégia Flank, com o oponente visto longe o bastante e à frente, sem
// investida e depois de um tempo do último flanco. O lado é o que tem mais espaço até a borda;
// sem localização, o lado que pede menos giro.
static bool escolheLadoFlanco() {
  if (P.arena.confiavel) {
    float le = arena_livre(P.op.angulo + FLANCO_DESVIO_DEG), ld = arena_livre(P.op.angulo - FLANCO_DESVIO_DEG);
    if (le < FLANCO_LIVRE_MIN_MM && ld < FLANCO_LIVRE_MIN_MM) return false;
    ladoFlanco = le >= ld ? 1 : -1;
  } else ladoFlanco = P.op.angulo >= 0 ? -1 : 1;
  return true;
}
static bool podeFlanquear() {
  if (estrategia != FLANK || !P.op.visto || investida()) return false;
  if (P.t_ms - tFimFlanco < 2000 && tFimFlanco) return false;
  float d = P.op.dist_mm > 0 ? P.op.dist_mm : P.rumo.dist_mm - OPONENTE_LADO_MM / 2;
  if (d < FLANCO_FECHA_MM + 60 || fabsf(P.op.angulo) > FLANCO_ABRE_DEG - 15) return false;
  return escolheLadoFlanco();
}

// O zigue-zague do Fan out vale com o oponente à vista, longe e na frente, sem investida e
// depois de um tempo do último.
static float distOponente() { return P.op.dist_mm > 0 ? P.op.dist_mm : P.rumo.dist_mm - OPONENTE_LADO_MM / 2; }
static bool podeLeque() {
  if (estrategia != FAN_OUT || !P.op.visto || investida()) return false;
  if (tFimLeque && P.t_ms - tFimLeque < 2000) return false;
  return distOponente() > LEQUE_FECHA_MM + 120 && fabsf(P.op.angulo) <= ABERTURA_ANG_MAX_DEG;
}

// Oponente colado atrás (só com os 7 sensores, que veem atrás): girar 180 graus na frente dele
// leva quase meio segundo e mostra a lateral, então o robô empurra de ré, com o mesmo torque.
static float erroRe() { float e = P.op.angulo - 180.0f; if (e < -180) e += 360; return e; }
static bool atrasColado() {
  return P.op.visto && fabsf(erroRe()) <= ANG_RE_DEG && P.rumo.dist_mm > 0 && P.rumo.dist_mm <= DIST_RE_MM;
}

// ToF colados (leitura abaixo do mínimo) na frente ou atrás: oponente encostado naquele lado
static uint16_t colados(bool frente) {
  uint16_t m = 0;
  for (uint8_t i = 0; i < N_TOF; i++)
    if ((frente && abs(TOF[i].ang) <= 30) || (!frente && abs(TOF[i].ang) >= 140)) m |= (1u << i);
  return P.op.colados & m;
}

// Empurrão em contato: atacando (de frente ou de ré) com o oponente encostado no lado do ataque.
static bool empurrando() {
  if (est == ATAQUE) return P.op.contato || colados(true);
  if (est == ATAQUE_RE) return colados(false) || (atrasColado() && P.rumo.dist_mm <= OPONENTE_LADO_MM / 2 + CONTATO_MM + 20);
  return false;
}

// Giro preso: na mira com o oponente aberto (giro no eixo ou curva fechada), a cinemática estima
// que o robô girou GIRO_TRAVA_DEG em T_GIRO_TRAVA_MS e o ângulo do oponente melhorou menos que
// GIRO_TRAVA_GANHO_DEG. Com o oponente encostado na lateral, a quina dele segura a do robô e o
// giro não acontece; sem contato, o oponente que gira junto (ou foge girando) também aparece
// assim, e a manobra curta não atrapalha.
static bool giroPreso() {
  if (est != MIRA || !P.op.visto || fabsf(P.op.angulo) <= ANG_MIRA_CURVA) { tGiroRef = 0; return false; }
  if (!tGiroRef) { tGiroRef = P.t_ms; angGiroRef = fabsf(P.op.angulo); giroRef = cinematica_giro(); return false; }
  if (P.t_ms - tGiroRef < T_GIRO_TRAVA_MS) return false;
  bool preso = fabsf(cinematica_giro() - giroRef) >= GIRO_TRAVA_DEG && angGiroRef - fabsf(P.op.angulo) < GIRO_TRAVA_GANHO_DEG;
  tGiroRef = P.t_ms; angGiroRef = fabsf(P.op.angulo); giroRef = cinematica_giro();
  return preso;
}

// Começo da manobra DESTRAVA. A quina do oponente segura a quina da frente do robô; uns 8 cm
// para trás ou para a frente já soltam as duas. Com espaço atrás (sem branco atrás há pouco, nenhuma
// ré anterior batendo na borda nos últimos 5 s e, com a localização confiável, mais de
// DESTRAVA_LIVRE_MM até a borda), o robô dá ré; sem espaço, avança reto, para dentro do dojô.
static void iniciaDestrava() {
  destravas++;
  ladoDestrava = P.op.angulo >= 0 ? 1 : -1;
  destravaComRe = !P.borda.tras && !oponente_borda_tras_recente() && !(tReBateu && P.t_ms - tReBateu < 5000) &&
                  (!P.arena.confiavel || arena_livre(180) > DESTRAVA_LIVRE_MM);
  Log.printf("# giro preso pelo oponente a %+.0f graus: %s\n", P.op.angulo, destravaComRe ? "re curta" : "avanco curto");
  entra(DESTRAVA);
}

// Escolha de estado com o oponente visto (comum à busca, à mira e ao fim do giro da borda).
static void decideComOponente() {
  if (atrasColado())        entra(ATAQUE_RE);
  else if (podeFlanquear()) entra(FLANCO);
  else if (podeLeque())     entra(LEQUE);
  else if (P.op.centrado)   entra(ATAQUE);
  else                      entra(MIRA);
}

// Abertura: o que cada estratégia faz quando os 5 s acabam com o oponente à vista (config.h).
static void abertura() {
  bool naFrente = fabsf(P.op.angulo) <= ABERTURA_ANG_MAX_DEG && !atrasColado();
  if (estrategia == FLANK && naFrente && escolheLadoFlanco()) { entra(FLANCO); aberturaFlanco = true; }
  else if (estrategia == FAN_OUT && naFrente)                entra(LEQUE);
  else                                                       decideComOponente();
}

// Tamanho do giro depois da borda. Com a localização confiável o robô gira até ficar de frente
// para o centro do dojô; sem ela, gira ANG_GIRO_BORDA para longe do lado da borda.
static void preparaGiro() {
  if (P.arena.confiavel) { alvoGiro = P.arena.ang; return; }
  int8_t sentido = ladoBorda != 0 ? ladoBorda : (P.op.lado_ultimo != 0 ? P.op.lado_ultimo : 1);
  alvoGiro = sentido * (float)ANG_GIRO_BORDA;
}

// Fuga da borda. Cada sensor confirmado no branco diz de que lado está a borda:
//   - só frente: RECUO; só trás: AVANCO_BORDA (nunca um giro para trás, porque atrás da faixa
//     não existe oponente para procurar);
//   - um lado inteiro (frente e trás do mesmo lado): SAIDA_LATERAL, uma curva para dentro com a
//     roda de dentro parada, que é o movimento que menos joga o canto de trás para fora;
//   - frente e trás em lados opostos (robô atravessado na faixa): giro direto para o centro.
static void decideFuga() {
  bool fe = false, fd = false, te = false, td = false;
  for (uint8_t i = 0; i < N_BORDA; i++) {
    if (!(P.borda.mascara & (1u << i))) continue;
    if (BORDA[i].frente) { if (BORDA[i].lado < 0) fe = true; else fd = true; }
    else                 { if (BORDA[i].lado < 0) te = true; else td = true; }
  }
  if (!P.borda.mascara) {                // sem máscara (não deveria acontecer): valem os resumos
    fe = P.borda.frente && P.borda.lado <= 0; fd = P.borda.frente && P.borda.lado >= 0;
    te = P.borda.tras && P.borda.lado <= 0;   td = P.borda.tras && P.borda.lado >= 0;
  }
  ladoBorda = P.borda.lado;
  bool frente = fe || fd, tras = te || td;
  if ((fe && te && !fd && !td) || (fd && td && !fe && !te)) { ladoBorda = fe ? -1 : 1; entra(SAIDA_LATERAL); }
  else if (frente && tras)                                  { entra(GIRO_BORDA); preparaGiro(); }
  else if (frente)                                          entra(RECUO);
  else                                                      entra(AVANCO_BORDA);
}

// Busca sem pista. Longe do centro (pela localização), o robô volta para o meio do dojô
// procurando no caminho. Perto do centro, cada estratégia procura do seu jeito: Rush e Flank
// giram no eixo para o lado do rumo (a seta do painel) com avanços curtos para mudar o ponto de
// vista; Fan out oscila o rumo de um lado para o outro e avança devagar.
static void busca(float dt) {
  int8_t s = P.rumo.angulo >= 0 ? 1 : -1;     // +1 gira para a esquerda, -1 para a direita
#if MODO_DEFENSIVO
  cinematica_comanda(0, s * GIRO_BUSCA_DPS);
  return;
#endif
  if (P.rumo.modo == RUMO_PROVAVEL) {          // o mapa ainda sabe onde o oponente deve estar
    if (fabsf(P.rumo.angulo) <= 12) cinematica_comanda(VEL_BUSCA_MMS, 0);
    else                            cinematica_comanda(0, s * GIRO_BUSCA_DPS);
    return;
  }
  if (P.arena.confiavel && P.arena.dist > RETORNO_DIST_MM) {   // volta ao centro do dojô
    float a = P.arena.ang;
    float w = constrain(K_MIRA * a, -(float)GIRO_MIRA_MAX_DPS, (float)GIRO_MIRA_MAX_DPS);
    float v = fabsf(a) <= 60 ? VEL_RETORNO_MMS * cosf(a * (float)DEG_TO_RAD) : 0;
    cinematica_comanda(v, w);
    return;
  }
  if (estrategia == FAN_OUT) {
    fanGiro += cinematica_w_dps() * dt;
    if (fanGiro * fanSentido >= FAN_AMPL_DEG) { fanSentido = -fanSentido; fanGiro = fanSentido * -FAN_AMPL_DEG; }
    bool espaco = !P.arena.confiavel || arena_livre(0) > FAN_LIVRE_MIN_MM;
    cinematica_comanda(espaco ? FAN_VEL_MMS : 0, fanSentido * FAN_GIRO_DPS);
    return;
  }
  if (buscaAvanca) {                             // avanço curto de reajuste de posição
    cinematica_comanda(VEL_BUSCA_MMS, 0);
    if (P.t_ms - tBusca >= T_BUSCA_AVANCO_MS) { buscaAvanca = false; tBusca = P.t_ms; }
    return;
  }
  cinematica_comanda(0, s * GIRO_BUSCA_DPS);     // giro no eixo para o lado do rumo
  if (P.rumo.modo == RUMO_PROCURAR && P.t_ms - tBusca >= T_BUSCA_GIRO_MS) { buscaAvanca = true; tBusca = P.t_ms; }
}

// Flanco: o rumo pedido é o ângulo do oponente mais FLANCO_DESVIO_DEG para o lado escolhido. Com
// o desvio constante, o caminho vira uma espiral que fecha na lateral do oponente; perto dele,
// ou com ele aberto demais, o robô ataca direto.
static void flanco(uint32_t t) {
  float d = distOponente();
  // na abertura o arco segue até passar do lado do oponente, mesmo com ele vindo rápido
  bool fim = aberturaFlanco
      ? (P.op.contato || d <= CONTATO_MM || fabsf(P.op.angulo) >= FLANCO_ABRE_DEG || t >= T_ABERTURA_FLANCO_MS)
      : (d <= FLANCO_FECHA_MM || fabsf(P.op.angulo) >= FLANCO_ABRE_DEG || t >= T_FLANCO_MAX_MS || investida());
  if (fim) {
    if (P.op.centrado) entra(ATAQUE); else entra(MIRA);
    return;
  }
  float alvo = P.op.angulo + ladoFlanco * (aberturaFlanco ? ABERTURA_FLANCO_DEG : FLANCO_DESVIO_DEG);
  float w = constrain(K_MIRA * alvo, -(float)GIRO_MIRA_MAX_DPS, (float)GIRO_MIRA_MAX_DPS);
  float c = cosf(alvo * (float)DEG_TO_RAD);
  cinematica_comanda(VEL_MAX_MMS * (c > 0.35f ? c : 0.35f), w);
}

// Começo da luta: estratégia e perfil travados e localização iniciada com o oponente visto
// durante a contagem (ele está parado na outra metade do dojô).
static void comecaLuta() {
  destravas = 0; tEmpurra = 0; tGiroRef = 0; avisouEmpurrao = false;
  arena_inicia(P.op.visto, P.op.x, P.op.y);
  arena_le(P.arena);
  Log.printf("# luta: estrategia %s, oponente %s%s\n", estrategia_nome(estrategia), escuro ? "ESCURO" : "CLARO",
             P.op.visto ? ", oponente visto na largada" : "");
}

void estados_passo() {
  uint32_t t = P.t_ms - tEntrada;      // tempo dentro do estado atual
  float dt = (P.t_ms - tAntPasso) * 0.001f;
  tAntPasso = P.t_ms;
  if (dt > 0.05f) dt = 0.05f;

  // A borda tem prioridade máxima em qualquer estado de luta (decideFuga). Durante a fuga a
  // borda continua vigiada: uma quina que volta ao branco no meio do giro (ou um empurrão do
  // oponente) leva de novo à fuga.
  bool emFuga = est == RECUO || est == AVANCO_BORDA || est == SAIDA_LATERAL ||
                (est == GIRO_BORDA && P.borda.frente && P.borda.tras);
  if (emLuta() && !emFuga && P.borda.detectada) {
    if (est == DESTRAVA && destravaComRe && P.borda.tras) tReBateu = P.t_ms | 1;
    decideFuga(); t = 0;
  }

  // Travamentos. O empurrão em contato não é interrompido (nas lutas simuladas, soltar no meio
  // dele entregou a luta), mas aparece no log e no painel e faz o modelo térmico contar a roda
  // parada. O giro preso vira a manobra DESTRAVA.
  bool emp = emLuta() && empurrando();
  if (emp) { if (!tEmpurra) tEmpurra = P.t_ms; }
  else { tEmpurra = 0; avisouEmpurrao = false; }
  if (emp && !avisouEmpurrao && P.t_ms - tEmpurra >= T_TRAVA_MS) {
    avisouEmpurrao = true;
    Log.printf("# empurrao travado ha %u ms (motor %.0f K acima do ambiente)\n", (unsigned)(P.t_ms - tEmpurra), cinematica_aquecimento_k());
  }
  bool preso = emLuta() && giroPreso();
  cinematica_rodas_travadas((emp && P.t_ms - tEmpurra >= 300) || est == DESTRAVA || preso);
  if (preso) { iniciaDestrava(); t = 0; }

  switch (est) {
    case ESPERA:
      // parado, a chave manda: estratégia e perfil do oponente seguem a DIP (o perfil vai para os sensores)
      estrategia = chave_estrategia();
      escuro = chave_escuro();
      tof_perfil(escuro);
      if (movAtivo && (P.borda.detectada || P.t_ms >= movFim)) {
        movAtivo = false;
        if (movBruto) Log.printf("# pwm: fim%s em %u ms\n", P.borda.detectada ? " (BORDA)" : "", (unsigned)(P.t_ms - movIni));
        else          Log.printf("# mov: fim%s em %u ms | estimado %.0f mm e %.0f graus\n", P.borda.detectada ? " (BORDA)" : "",
                                 (unsigned)(P.t_ms - movIni), cinematica_dist(), cinematica_giro());
      }
      if (movAtivo && movBruto) motores(movE, movD);
      else if (movAtivo)        cinematica_comanda(movV, movW);
      else                      cinematica_para();
      // LED aceso fixo indica robô pronto; piscando rápido indica que faltam sensores
      led(P.saude.pronto || (P.t_ms / 100) % 2);
      if (P.start || pedidoStart) {
        pedidoStart = false;
        movAtivo = false;
        if (!P.saude.pronto)
          Log.printf("! NAO INICIA: so %u de %u sensores de oponente OK (minimo %d). Rode 'lista'.\n",
                     P.saude.tof_ok, P.saude.tof_total, TOF_MINIMO_PARA_LUTAR);
        else if (tof_perfil_escuro() != escuro)
          Log.println(F("! NAO INICIA: perfil do oponente ainda sendo aplicado nos sensores, aperte de novo"));
        else {
          if (chave_conflito()) Log.println(F("! chave DIP com mais de uma estrategia ligada: vale a de menor numero"));
          entra(CONTAGEM);
        }
      }
      break;

    case CONTAGEM:                                   // regra da CoRA: 5 s parado
      cinematica_para();
      led((t / 250) % 2);                            // LED piscando devagar durante a contagem
      if (t >= T_CONTAGEM_MS) {
        comecaLuta();
        entra(BUSCA);
        if (P.op.visto) abertura();                   // quem já está à vista recebe a abertura da estratégia
      }
      break;

    case BUSCA:
      led(false);                                    // em luta o LED fica apagado
      if (P.op.visto) { decideComOponente(); break; }
      busca(dt);
      break;

    case MIRA:
      if (!P.op.visto)    { if (P.op.contato) { entra(ATAQUE); break; } entra(BUSCA); break; }
      if (atrasColado())  { entra(ATAQUE_RE); break; }
      if (podeFlanquear()) { entra(FLANCO); break; }
      if (podeLeque())     { entra(LEQUE); break; }
      if (P.op.centrado)  { entra(ATAQUE); break; }
      mira(P.op.angulo);                             // curva até ANG_MIRA_CURVA, giro no eixo acima disso
      break;

    case FLANCO:
      if (!P.op.visto) { entra(BUSCA); break; }
      flanco(t);
      break;

    case LEQUE: {                                    // Fan out: chegada em zigue-zague
      if (!P.op.visto) { entra(BUSCA); break; }
      if (atrasColado()) { entra(ATAQUE_RE); break; }
      if (P.op.contato || distOponente() <= LEQUE_FECHA_MM || fabsf(P.op.angulo) > LEQUE_ANG_MAX_DEG || t >= T_LEQUE_MAX_MS) {
        if (P.op.centrado) entra(ATAQUE); else entra(MIRA);
        break;
      }
      int8_t lado = ((t / T_LEQUE_MEIO_MS) % 2) ? -1 : 1;
      float alvo = P.op.angulo + lado * LEQUE_DESVIO_DEG;
      float w = constrain(K_MIRA * alvo, -(float)GIRO_MIRA_MAX_DPS, (float)GIRO_MIRA_MAX_DPS);
      float c = cosf(alvo * (float)DEG_TO_RAD);
      cinematica_comanda(VEL_MAX_MMS * LEQUE_VEL * (c > 0.35f ? c : 0.35f), w);
      break;
    }

    case ATAQUE_RE: {                                // empurrão de ré com o oponente colado atrás
      // Colado na traseira, o ToF de trás também pode perder a leitura: o empurrão continua por
      // T_SEGURA_CONTATO_MS depois da última vez em que o oponente foi visto ali.
      float e = erroRe();
      if (atrasColado()) tReVisto = P.t_ms;
      bool segura = P.t_ms - tReVisto <= T_SEGURA_CONTATO_MS;
      if (!P.op.visto && !segura) { entra(BUSCA); break; }
      if (P.op.visto && (fabsf(e) > ANG_RE_DEG + 15 || P.rumo.dist_mm > DIST_RE_MM + 80)) { entra(MIRA); break; }
      cinematica_comanda(-VEL_MAX_MMS, P.op.visto ? K_MIRA * e : 0);
      break;
    }

    case ATAQUE:
      // Com o oponente colado na frente, o ToF pode perder a leitura por um instante: o empurrão
      // continua (P.op.contato) e o robô só volta a mirar quando o alvo sai do centro com folga.
      if (!P.op.visto && !P.op.contato) { entra(BUSCA); break; }
      if (P.op.visto && !P.op.centrado && fabsf(P.op.angulo) > CENTRO_DEG + 6 && !P.op.contato) { entra(MIRA); break; }
#if MODO_DEFENSIVO
      cinematica_para();                             // no modo defensivo a lâmina fica parada no chão
#else
      // empurrão na velocidade máxima, com a mesma correção de giro da mira para o oponente
      // não escorregar para fora do centro (até CENTRO_DEG o giro pedido é pequeno)
      cinematica_comanda(VEL_MAX_MMS, P.op.visto ? K_MIRA * P.op.angulo : 0);
#endif
      break;

    case DESTRAVA:                                   // solta a quina presa e volta a mirar
      if (t < (destravaComRe ? T_DESTRAVA_RE_MS : T_DESTRAVA_AVANCO_MS))
        cinematica_comanda(destravaComRe ? -VEL_MAX_MMS : VEL_MAX_MMS, 0);
      else { entra(BUSCA); if (P.op.visto) decideComOponente(); }
      break;

    case RECUO: {                                    // o robô recua enquanto a frente vê branco
      // Recuo que não tira a frente do branco em T_RECUO_BLOQ_MS indica o robô preso entre a
      // borda e o oponente (ou patinando): o recuo vira uma curva de ré, que desencaixa a traseira
      // e vira a frente para o centro do dojô (ou para longe do lado da borda).
      float w = 0;
      if (t >= T_RECUO_BLOQ_MS && P.borda.frente) {
        int8_t s = P.arena.confiavel ? (P.arena.ang >= 0 ? 1 : -1) : (ladoBorda != 0 ? -ladoBorda : 1);
        w = s * GIRO_BORDA_DPS * 0.6f;
      }
      cinematica_comanda(-VEL_MAX_MMS, w, PERFIL_BORDA);
      if (P.borda.tras) { entra(GIRO_BORDA); preparaGiro(); break; }   // a traseira chegou na borda: fim do recuo
      if (P.borda.frente) tBordaLimpa = P.t_ms;       // ainda no branco: o tempo é renovado
      else if (P.t_ms - tBordaLimpa >= T_RECUO_EXTRA_MS) {
        if (atrasColado()) entra(ATAQUE_RE);          // o recuo já está empurrando o oponente de trás: continua
        else { entra(GIRO_BORDA); preparaGiro(); }
      }
      break;
    }

    case AVANCO_BORDA:                               // o robô avança enquanto a traseira vê branco
      cinematica_comanda(VEL_MAX_MMS, 0, PERFIL_BORDA);
      if (P.borda.frente) { entra(GIRO_BORDA); preparaGiro(); break; } // a frente também chegou: hora de girar
      if (P.borda.tras) tBordaLimpa = P.t_ms;
      else if (P.t_ms - tBordaLimpa >= T_RECUO_EXTRA_MS) entra(BUSCA);  // já de frente para dentro do dojô
      break;

    case SAIDA_LATERAL: {                            // borda em um lado: curva para dentro, roda de dentro parada
      // ladoBorda -1 é a borda na esquerda: a roda esquerda vai para a frente e a direita fica
      // parada, e o robô vira para a direita (giro negativo, para dentro)
      float w = ladoBorda * VEL_MAX_MMS / BITOLA_MM * (float)RAD_TO_DEG;
      cinematica_comanda(VEL_MAX_MMS / 2, w, PERFIL_BORDA);
      bool lado = false;
      for (uint8_t i = 0; i < N_BORDA; i++)
        if ((P.borda.mascara & (1u << i)) && BORDA[i].lado == ladoBorda) lado = true;
      if (lado) tBordaLimpa = P.t_ms;
      else if (P.t_ms - tBordaLimpa >= T_RECUO_EXTRA_MS) { entra(GIRO_BORDA); preparaGiro(); }
      else if (P.borda.frente && !lado) { entra(RECUO); }
      break;
    }

    case GIRO_BORDA: {                               // giro para o centro, vigiando o oponente
      // O giro acaba quando a cinemática estima o ângulo pedido, e não por um tempo fixo;
      // T_GIRO_BORDA_MAX_MS é só a segurança para o caso de a estimativa não chegar lá.
      int8_t sentido = alvoGiro >= 0 ? 1 : -1;
      cinematica_comanda(0, sentido * GIRO_BORDA_DPS);
      if (P.op.visto)                                          decideComOponente();
      else if (fabsf(cinematica_giro()) >= fabsf(alvoGiro) - 10 || t >= T_GIRO_BORDA_MAX_MS) entra(BUSCA);
      break;
    }

    default:
      entra(ESPERA);                                 // estado inválido: retorno ao início por segurança
  }
}
