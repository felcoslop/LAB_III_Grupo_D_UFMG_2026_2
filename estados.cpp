// estados.cpp
// Implementação da máquina de estados, com as regras do Blackbook:
//   capítulo 2: o robô só ataca quando o sensor central vê o alvo centrado; nos outros casos ele mira;
//   capítulo 10: a decisão usa onde o oponente foi visto por último, o recuo de borda dura
//   enquanto o sensor vê branco e mais uma folga de 10 ms, o giro de recuperação continua
//   vigiando o oponente e o modo defensivo nunca avança;
//   capítulo 9: a borda chega aqui já confirmada pelo filtro de tempo do borda.cpp.
// A função estados_passo() é chamada pelo loop() do sumo_esp32.ino logo depois da percepção.

#include "estados.h"
#include "config.h"
#include "telemetria.h"
#include "percepcao.h"
#include "motores.h"

static Estado   est = ESPERA;
static uint32_t tEntrada = 0;          // momento em que o estado atual começou
static uint32_t tBordaLimpa = 0;       // última vez em que a borda ainda estava sendo vista
static int8_t   ladoBorda = 0;         // lado da borda que disparou o desvio (-1, 0 ou +1)
static bool     pedidoStart = false;   // start pedido pelo comando "go" (debug ou painel)
static bool     buscaAvanca = false;   // na BUSCA, true indica o avanço curto que muda o ponto de vista
static uint32_t tBusca = 0;            // início do giro ou do avanço atual da BUSCA

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
  tBordaLimpa = P.t_ms;
}

// Os estados de luta são todos os que vêm depois da CONTAGEM na ordem do enum.
static bool emLuta() { return est >= BUSCA; }

void estados_init()    { pinMode(PINO_LED, OUTPUT); est = ESPERA; tEntrada = millis(); }
void estados_iniciar() { pedidoStart = true; }
void estados_parar()   { entra(ESPERA); motores_para(); }

void estados_passo() {
  uint32_t t = P.t_ms - tEntrada;      // tempo dentro do estado atual

  // A borda tem prioridade máxima em qualquer estado de luta:
  // frente no branco vai para RECUO, trás no branco vai para AVANCO_BORDA e frente e trás
  // ao mesmo tempo (robô de lado na linha) vai direto para GIRO_BORDA.
  if (emLuta() && est != RECUO && est != GIRO_BORDA && est != AVANCO_BORDA && P.borda.detectada) {
    ladoBorda = P.borda.lado;
    if (P.borda.frente && P.borda.tras) entra(GIRO_BORDA);
    else if (P.borda.frente)            entra(RECUO);
    else                                entra(AVANCO_BORDA);
    t = 0;
  }

  switch (est) {
    case ESPERA:
      motores_para();
      // LED aceso fixo indica robô pronto; piscando rápido indica que faltam sensores
      digitalWrite(PINO_LED, P.saude.pronto ? HIGH : (P.t_ms / 100) % 2);
      if (P.start || pedidoStart) {
        pedidoStart = false;
        if (P.saude.pronto) entra(CONTAGEM);
        else Log.printf("! NAO INICIA: so %u de %u sensores de oponente OK (minimo %d). Rode 'lista'.\n",
                           P.saude.tof_ok, P.saude.tof_total, TOF_MINIMO_PARA_LUTAR);
      }
      break;

    case CONTAGEM:                                   // regra RoboCore: 5 s parado
      motores_para();
      digitalWrite(PINO_LED, (t / 250) % 2);         // LED piscando devagar durante a contagem
      if (t >= T_CONTAGEM_MS) entra(BUSCA);
      break;

    case BUSCA:
      // O robô segue o rumo calculado no percepcao.cpp (a seta do painel): gira para onde o
      // oponente provavelmente está ou, sem pista, para o lado em que os cones varrem os pontos
      // cegos. Depois de T_BUSCA_GIRO_MS girando sem achar nada, o robô avança um pouco para
      // mudar o ponto de vista e recomeça o giro.
      digitalWrite(PINO_LED, LOW);                   // em luta o LED fica apagado
      if (P.op.centrado)   { entra(ATAQUE); break; }
      if (P.op.visto)      { entra(MIRA);   break; }
      if (buscaAvanca) {                             // avanço curto de reajuste de posição
        motores(VEL_BUSCA, VEL_BUSCA);
        if (P.t_ms - tBusca >= T_BUSCA_AVANCO_MS) { buscaAvanca = false; tBusca = P.t_ms; }
        break;
      }
      {
        int8_t s = P.rumo.angulo >= 0 ? 1 : -1;     // +1 gira para a esquerda, -1 para a direita
#if MODO_DEFENSIVO
        motores(-s * VEL_BUSCA, s * VEL_BUSCA);                          // o robô só gira no eixo
#else
        if (P.rumo.modo == RUMO_PROVAVEL && fabsf(P.rumo.angulo) <= 12)
          motores(VEL_BUSCA, VEL_BUSCA);                                 // provável bem à frente: o robô chega mais perto
        else
          motores(-s * VEL_BUSCA, s * VEL_BUSCA);                        // giro no eixo para o lado do rumo
        if (P.rumo.modo == RUMO_PROCURAR && P.t_ms - tBusca >= T_BUSCA_GIRO_MS) { buscaAvanca = true; tBusca = P.t_ms; }
#endif
      }
      break;

    case MIRA:
      if (!P.op.visto)    { entra(BUSCA);  break; }
      if (P.op.centrado)  { entra(ATAQUE); break; }
      if (fabsf(P.op.angulo) <= 40) {                // oponente na diagonal: correção em curva
#if MODO_DEFENSIVO
        if (P.op.angulo > 0) motores(-VEL_MIRA_IN, VEL_MIRA_IN); else motores(VEL_MIRA_IN, -VEL_MIRA_IN);
#else
        if (P.op.angulo > 0) motores(VEL_MIRA_IN, VEL_ATAQUE); else motores(VEL_ATAQUE, VEL_MIRA_IN);
#endif
      } else {                                       // oponente na lateral: giro no eixo
        if (P.op.angulo > 0) motores(-VEL_GIRO, VEL_GIRO); else motores(VEL_GIRO, -VEL_GIRO);
      }
      break;

    case ATAQUE:
      if (!P.op.visto)    { entra(BUSCA); break; }
      if (!P.op.centrado) { entra(MIRA);  break; }
#if MODO_DEFENSIVO
      motores_para();                                // no modo defensivo a lâmina fica parada no chão
#else
      motores(VEL_ATAQUE, VEL_ATAQUE);
#endif
      break;

    case RECUO:                                      // o robô recua enquanto a frente vê branco
      motores(-VEL_RECUO, -VEL_RECUO);
      if (P.borda.tras) { entra(GIRO_BORDA); break; }                  // a traseira chegou na borda: fim do recuo
      if (P.borda.frente) tBordaLimpa = P.t_ms;       // ainda no branco: o tempo é renovado
      else if (P.t_ms - tBordaLimpa >= T_RECUO_EXTRA_MS) entra(GIRO_BORDA);
      break;

    case AVANCO_BORDA:                               // o robô avança enquanto a traseira vê branco
      motores(VEL_RECUO, VEL_RECUO);
      if (P.borda.frente) { entra(GIRO_BORDA); break; }                // a frente também chegou: hora de girar
      if (P.borda.tras) tBordaLimpa = P.t_ms;
      else if (P.t_ms - tBordaLimpa >= T_RECUO_EXTRA_MS) entra(BUSCA);
      break;

    case GIRO_BORDA: {                               // giro para longe da borda, vigiando o oponente
      // sentido +1 gira para a esquerda. Com a borda na esquerda (lado -1) o robô gira para a
      // direita (-1) e o contrário também vale. Sem lado definido, vale o último lado do oponente.
      int8_t sentido = ladoBorda != 0 ? ladoBorda : (P.op.lado_ultimo != 0 ? P.op.lado_ultimo : 1);
      motores(-sentido * VEL_GIRO, sentido * VEL_GIRO);
      if (P.op.centrado)          entra(ATAQUE);
      else if (P.op.visto)        entra(MIRA);
      else if (t >= T_GIRO_BORDA_MS) entra(BUSCA);
      break;
    }

    default:
      entra(ESPERA);                                 // estado inválido: retorno ao início por segurança
  }
}
