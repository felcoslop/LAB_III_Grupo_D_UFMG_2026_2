// antitrava.cpp
// Detecção do empurrão travado pela caixa do rastreador e manobras para sair dele (antitrava.h).
#include "antitrava.h"
#if ANTI_TRAVA
#include "percepcao.h"
#include "cinematica.h"
#include "arena.h"
#include "oponente.h"
#include "telemetria.h"

static uint8_t  manobra = ANTI_TRAVA_MANOBRA;   // manobra escolhida (comando "trava N")
static uint8_t  emCurso = MT_NENHUMA;           // manobra em andamento
static bool     ativa = false;
static uint32_t tIni = 0;                       // começo da manobra em andamento
static uint32_t tPausa = 0;                     // fim da última manobra
static uint32_t tRef = 0;                       // começo da janela com a caixa parada (0 sem janela)
static uint32_t tFora = 0;                      // começo da saída da caixa da tolerância (0 dentro)
static float    dRef = 0, aRef = 0;             // caixa no começo da janela (mm, graus)
static int8_t   lado = 1;                       // +1 esquerda, -1 direita
static uint16_t disparos = 0;

static const char* NOMES[MT_N] = {"nenhuma", "lateral", "torque", "zigue", "giro", "pulso"};

const char* antitrava_nome(uint8_t m) { return m < MT_N ? NOMES[m] : "?"; }
uint8_t     antitrava_manobra()       { return manobra; }
void        antitrava_define(uint8_t m) { if (m < MT_N) manobra = m; }
bool        antitrava_ativa()         { return ativa; }
uint16_t    antitrava_disparos()      { return disparos; }
uint32_t    antitrava_parado_ms()     { return tRef ? P.t_ms - tRef : 0; }

void antitrava_inicia_luta() { ativa = false; tRef = 0; tPausa = 0; disparos = 0; }
void antitrava_aborta()      { if (ativa) { ativa = false; tPausa = P.t_ms | 1; } tRef = 0; }

static float difAng(float a, float b) {
  float d = a - b;
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

// Lado com mais espaço até a borda (localização no dojô); sem localização confiável, o lado
// contrário ao deslocamento da caixa, que tira a frente do robô de baixo do empurrão.
static int8_t ladoLivre() {
  if (P.arena.confiavel) return arena_livre(90) >= arena_livre(-90) ? 1 : -1;
  return P.op.angulo >= 0 ? -1 : 1;
}

// Espaço atrás para a ré da manobra lateral (as mesmas condições da DESTRAVA)
static bool espacoAtras() {
  return !P.borda.tras && !oponente_borda_tras_recente() && (!P.arena.confiavel || arena_livre(180) > ANTI_TRAVA_LIVRE_MM);
}

static void termina() { ativa = false; tPausa = P.t_ms | 1; tRef = 0; }

// Uma volta da manobra em andamento; false quando ela acabou (o estado volta a comandar).
static bool executa() {
  uint32_t t = P.t_ms - tIni;
  switch (emCurso) {
    case MT_LATERAL:
      if (t < ANTI_TRAVA_RE_MS) { cinematica_comanda(-VEL_MAX_MMS, lado * (float)ANTI_TRAVA_RE_GIRO_DPS); return true; }
      if (t < ANTI_TRAVA_RE_MS + ANTI_TRAVA_ARCO_MS && !(P.op.contato && t > ANTI_TRAVA_RE_MS + 100)) {
        // arco pela lateral do oponente, como o FLANCO: rumo no ângulo dele mais o desvio
        float alvo = P.op.angulo + lado * (float)ANTI_TRAVA_ARCO_DEG;
        float w = constrain(K_MIRA * alvo, -(float)GIRO_MIRA_MAX_DPS, (float)GIRO_MIRA_MAX_DPS);
        float c = cosf(alvo * (float)DEG_TO_RAD);
        cinematica_comanda(VEL_MAX_MMS * (c > 0.35f ? c : 0.35f), w);
        return true;
      }
      break;
    case MT_TORQUE:
      if (t < ANTI_TRAVA_TORQUE_MS) { cinematica_comanda(VEL_MAX_MMS, lado * (float)ANTI_TRAVA_TORQUE_DPS); return true; }
      break;
    case MT_ZIGUE:
      if (t < ANTI_TRAVA_TORQUE_MS) {
        int8_t s = ((t / ANTI_TRAVA_ZIGUE_MS) % 2) ? -lado : lado;
        cinematica_comanda(VEL_MAX_MMS, s * (float)ANTI_TRAVA_TORQUE_DPS);
        return true;
      }
      break;
    case MT_GIRO:
      if (t < ANTI_TRAVA_GIRO_MS) { cinematica_comanda(0, lado * GIRO_MAX_DPS); return true; }
      break;
    case MT_PULSO:
      // três ciclos: solta as rodas por um instante (o oponente empurra o robô para trás, o peso
      // vai para a rampa e a ponta desce) e volta a empurrar pelo próprio ATAQUE
      if (t < 3 * ANTI_TRAVA_PULSO_MS) {
        if (t % ANTI_TRAVA_PULSO_MS < ANTI_TRAVA_SOLTA_MS) { cinematica_para(); return true; }
        return false;
      }
      break;
    default:
      break;
  }
  termina();
  return false;
}

bool antitrava_passo(bool podeAgir) {
  if (ativa) {
    if (!podeAgir) { termina(); return false; }
    // o pulso devolve o comando ao ATAQUE entre as soltas sem acabar a manobra
    return executa();
  }
  bool perto = (P.op.visto || P.op.contato) &&
               (P.op.contato || (P.op.dist_mm >= 0 && P.op.dist_mm <= ANTI_TRAVA_PERTO_MM));
  if (!podeAgir || !perto || (tPausa && P.t_ms - tPausa < ANTI_TRAVA_PAUSA_MS)) { tRef = 0; return false; }
  float d = hypotf(P.op.x, P.op.y), a = atan2f(P.op.y, P.op.x) * (float)RAD_TO_DEG;
  if (!tRef) { tRef = P.t_ms; dRef = d; aRef = a; tFora = 0; return false; }
  // um salto curto da estimativa (leitura colada no contato) não zera a janela; fora da
  // tolerância por mais de ANTI_TRAVA_FORA_MS, a caixa andou e a janela recomeça ali
  if (fabsf(d - dRef) > ANTI_TRAVA_TOL_MM || fabsf(difAng(a, aRef)) > ANTI_TRAVA_TOL_DEG) {
    if (!tFora) tFora = P.t_ms | 1;
    if (P.t_ms - tFora >= ANTI_TRAVA_FORA_MS) { tRef = P.t_ms; dRef = d; aRef = a; tFora = 0; }
    return false;
  }
  tFora = 0;
  if (P.t_ms - tRef < ANTI_TRAVA_T_MS) return false;
  disparos++;
  lado = ladoLivre();
  emCurso = manobra;
  if (emCurso == MT_LATERAL && !espacoAtras()) emCurso = MT_TORQUE;   // sem espaço atrás, a ré entregaria a luta
  Log.printf("# caixa parada ha %u ms a %.0f mm e %+.0f graus: manobra %s para a %s\n", (unsigned)(P.t_ms - tRef), d, a,
             antitrava_nome(emCurso), lado > 0 ? "esquerda" : "direita");
  if (emCurso == MT_NENHUMA) { tRef = 0; tPausa = P.t_ms | 1; return false; }
  ativa = true;
  tIni = P.t_ms;
  return executa();
}

#endif  // ANTI_TRAVA
