// estados.cpp
// Implementação da máquina de estados, com as regras do Blackbook:
//   capítulo 2: o robô só ataca quando o sensor central vê o alvo centrado; nos outros casos ele mira;
//   capítulo 10: a decisão usa onde o oponente foi visto por último, o recuo de borda dura
//   enquanto o sensor vê branco e mais uma folga de 10 ms, o giro de recuperação continua
//   vigiando o oponente e o modo defensivo nunca avança;
//   capítulo 9: a borda chega aqui já confirmada pelo filtro de tempo do borda.cpp.
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

static Estado   est = ESPERA;
static uint32_t tEntrada = 0;          // momento em que o estado atual começou
static uint32_t tBordaLimpa = 0;       // última vez em que a borda ainda estava sendo vista
static int8_t   ladoBorda = 0;         // lado da borda que disparou o desvio (-1, 0 ou +1)
static bool     pedidoStart = false;   // start pedido pelo comando "go" (debug ou painel)
static bool     buscaAvanca = false;   // na BUSCA, true indica o avanço curto que muda o ponto de vista
static uint32_t tBusca = 0;            // início do giro ou do avanço atual da BUSCA
// movimento de teste do comando "mov", só em ESPERA (Blackbook, capítulo 11: medir as primitivas)
static bool     movAtivo = false;
static bool     movBruto = false;      // true no teste pwm: PWM direto, sem passar pela cinemática
static float    movV = 0, movW = 0;
static int16_t  movE = 0, movD = 0;
static uint32_t movFim = 0, movIni = 0;

static const char* NOMES[N_ESTADOS] = {"ESPERA", "CONTAGEM", "BUSCA", "MIRA", "ATAQUE", "RECUO", "GIRO_BORDA", "AVANCO_BORDA"};

const char* estado_nome(Estado e) { return e < N_ESTADOS ? NOMES[e] : "?"; }
Estado      estado_atual()        { return est; }

// Troca de estado. Toda troca aparece no log e reinicia os tempos usados pelos estados.
static void entra(Estado novo) {
  if (novo == est) return;
  Log.printf("# estado: %s -> %s\n", NOMES[est], NOMES[novo]);
  est = novo;
  tEntrada = P.t_ms;
  if (novo == BUSCA) { buscaAvanca = false; tBusca = P.t_ms; }
  if (novo == GIRO_BORDA) cinematica_zera_odometria();   // o giro de recuperação é medido a partir daqui
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
  est = ESPERA; tEntrada = millis();
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

void estados_passo() {
  uint32_t t = P.t_ms - tEntrada;      // tempo dentro do estado atual

  // A borda tem prioridade máxima em qualquer estado de luta:
  // frente no branco vai para RECUO, trás no branco vai para AVANCO_BORDA e frente e trás
  // ao mesmo tempo (robô de lado na linha) vai direto para GIRO_BORDA. Durante o GIRO_BORDA a
  // borda continua vigiada: uma quina que volta ao branco no meio do giro (ou um empurrão do
  // oponente) leva de novo ao RECUO ou ao AVANCO_BORDA.
  bool emFuga = est == RECUO || est == AVANCO_BORDA || (est == GIRO_BORDA && P.borda.frente && P.borda.tras);
  if (emLuta() && !emFuga && P.borda.detectada) {
    ladoBorda = P.borda.lado;
    if (P.borda.frente && P.borda.tras) entra(GIRO_BORDA);
    else if (P.borda.frente)            entra(RECUO);
    else                                entra(AVANCO_BORDA);
    t = 0;
  }

  switch (est) {
    case ESPERA:
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
        if (P.saude.pronto) entra(CONTAGEM);
        else Log.printf("! NAO INICIA: so %u de %u sensores de oponente OK (minimo %d). Rode 'lista'.\n",
                           P.saude.tof_ok, P.saude.tof_total, TOF_MINIMO_PARA_LUTAR);
      }
      break;

    case CONTAGEM:                                   // regra RoboCore: 5 s parado
      cinematica_para();
      led((t / 250) % 2);                            // LED piscando devagar durante a contagem
      if (t >= T_CONTAGEM_MS) entra(BUSCA);
      break;

    case BUSCA:
      // O robô segue o rumo calculado no percepcao.cpp (a seta do painel): gira para onde o
      // oponente provavelmente está ou, sem pista, para o lado em que os cones varrem os pontos
      // cegos. Depois de T_BUSCA_GIRO_MS girando sem achar nada, o robô avança um pouco para
      // mudar o ponto de vista e recomeça o giro.
      led(false);                                    // em luta o LED fica apagado
      if (P.op.centrado)   { entra(ATAQUE); break; }
      if (P.op.visto)      { entra(MIRA);   break; }
      if (buscaAvanca) {                             // avanço curto de reajuste de posição
        cinematica_comanda(VEL_BUSCA_MMS, 0);
        if (P.t_ms - tBusca >= T_BUSCA_AVANCO_MS) { buscaAvanca = false; tBusca = P.t_ms; }
        break;
      }
      {
        int8_t s = P.rumo.angulo >= 0 ? 1 : -1;     // +1 gira para a esquerda, -1 para a direita
#if MODO_DEFENSIVO
        cinematica_comanda(0, s * GIRO_BUSCA_DPS);                       // o robô só gira no eixo
#else
        if (P.rumo.modo == RUMO_PROVAVEL && fabsf(P.rumo.angulo) <= 12)
          cinematica_comanda(VEL_BUSCA_MMS, 0);                          // provável bem à frente: o robô chega mais perto
        else
          cinematica_comanda(0, s * GIRO_BUSCA_DPS);                     // giro no eixo para o lado do rumo
        if (P.rumo.modo == RUMO_PROCURAR && P.t_ms - tBusca >= T_BUSCA_GIRO_MS) { buscaAvanca = true; tBusca = P.t_ms; }
#endif
      }
      break;

    case MIRA:
      if (!P.op.visto)    { entra(BUSCA);  break; }
      if (P.op.centrado)  { entra(ATAQUE); break; }
      mira(P.op.angulo);                             // curva até ANG_MIRA_CURVA, giro no eixo acima disso
      break;

    case ATAQUE:
      if (!P.op.visto)    { entra(BUSCA); break; }
      if (!P.op.centrado) { entra(MIRA);  break; }
#if MODO_DEFENSIVO
      cinematica_para();                             // no modo defensivo a lâmina fica parada no chão
#else
      // empurrão na velocidade máxima, com a mesma correção de giro da mira para o oponente
      // não escorregar para fora do centro (até CENTRO_DEG o giro pedido é pequeno)
      cinematica_comanda(VEL_MAX_MMS, K_MIRA * P.op.angulo);
#endif
      break;

    case RECUO:                                      // o robô recua enquanto a frente vê branco
      cinematica_comanda(-VEL_MAX_MMS, 0, PERFIL_BORDA);
      if (P.borda.tras) { entra(GIRO_BORDA); break; }                  // a traseira chegou na borda: fim do recuo
      if (P.borda.frente) tBordaLimpa = P.t_ms;       // ainda no branco: o tempo é renovado
      else if (P.t_ms - tBordaLimpa >= T_RECUO_EXTRA_MS) entra(GIRO_BORDA);
      break;

    case AVANCO_BORDA:                               // o robô avança enquanto a traseira vê branco
      cinematica_comanda(VEL_MAX_MMS, 0, PERFIL_BORDA);
      if (P.borda.frente) { entra(GIRO_BORDA); break; }                // a frente também chegou: hora de girar
      if (P.borda.tras) tBordaLimpa = P.t_ms;
      else if (P.t_ms - tBordaLimpa >= T_RECUO_EXTRA_MS) entra(BUSCA);
      break;

    case GIRO_BORDA: {                               // giro para longe da borda, vigiando o oponente
      // sentido +1 gira para a esquerda. Com a borda na esquerda (lado -1) o robô gira para a
      // direita (-1) e o contrário também vale. Sem lado definido, vale o último lado do oponente.
      int8_t sentido = ladoBorda != 0 ? ladoBorda : (P.op.lado_ultimo != 0 ? P.op.lado_ultimo : 1);
      // O giro acaba quando a cinemática estima ANG_GIRO_BORDA graus, e não por um tempo fixo;
      // T_GIRO_BORDA_MAX_MS é só a segurança para o caso de a estimativa não chegar lá.
      cinematica_comanda(0, sentido * GIRO_BORDA_DPS);
      if (P.op.centrado)          entra(ATAQUE);
      else if (P.op.visto)        entra(MIRA);
      else if (fabsf(cinematica_giro()) >= ANG_GIRO_BORDA || t >= T_GIRO_BORDA_MAX_MS) entra(BUSCA);
      break;
    }

    default:
      entra(ESPERA);                                 // estado inválido: retorno ao início por segurança
  }
}
