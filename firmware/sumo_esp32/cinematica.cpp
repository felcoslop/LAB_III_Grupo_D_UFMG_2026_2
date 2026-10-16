// cinematica.cpp
// Implementação da cinemática das rodas e do modelo do acionamento (detalhes no cinematica.h).
// Referências das partes já validadas em outros projetos:
//   - cinemática inversa e limitador de velocidade e aceleração: diff_drive_controller e
//     SpeedLimiter do ros2_controllers (PAL Robotics e ros-controls, licença Apache 2.0);
//   - saturação das rodas pela rotação máxima do motor: Kinematics::calculateRPM do
//     linorobot2_hardware (Juan Miguel Jimeno, licença Apache 2.0);
//   - feed-forward kS, kV e kA: SimpleMotorFeedforward do WPILib (licença BSD).
// O código abaixo foi escrito para este projeto a partir dessas ideias. As mudanças para o sumô
// são três: a saturação dá prioridade ao giro (o linorobot reduz avanço e giro na mesma
// proporção), o perfil da borda pula o limitador e usa a tensão toda, e o modelo do motor
// também estima a velocidade real das rodas, porque o JGB37-520 deste projeto não tem encoder.

#include "cinematica.h"
#include "motores.h"

// O robô nunca pode cair: a distância de parada no pior caso precisa caber entre o sensor de
// borda e o ponto de apoio das rodas, somada à faixa branca. Um motor mais rápido, uma roda
// maior ou uma bateria com mais tensão que quebrem essa conta param a compilação aqui.
static_assert(DIST_PARADA_MM < BORDA_FAIXA_MM + BORDA_ATE_RODA_MM,
              "distancia de parada na borda maior que a margem ate as rodas: reduza a velocidade maxima");

static float    vCmd = 0, wCmd = 0;       // comando depois do limitador (mm/s e rad/s)
static float    rodaE = 0, rodaD = 0;     // velocidade estimada de cada roda (mm/s)
static float    giroAcum = 0;             // giro estimado desde o último zera (graus)
static float    distAcum = 0;             // avanço estimado desde o último zera (mm)
static uint32_t tAnt = 0;                 // momento da chamada anterior (micros)
static float    aquecE = 0, aquecD = 0;   // elevação de temperatura estimada de cada motor (K)
static bool     travadas = false;         // rodas travadas, avisado pela máquina de estados
static bool     quente = false;           // limite térmico ligado

// Corrente e aquecimento de um motor num intervalo dt: corrente = (tensão no motor - força
// contraeletromotriz) / resistência, com a força contraeletromotriz kV.v (zero com a roda
// travada); calor I²R; resfriamento proporcional à elevação.
static float aquece(float elev, float u, float vRoda, float dt) {
  const float R = MOTOR_V_NOMINAL / MOTOR_I_TRAVADO_A;
  float vm = u * CIN_V_DISP, fcem = travadas ? 0.0f : CIN_KV * vRoda;
  float i = (vm - fcem) / R;
  if (vm * i < 0) i = 0;                   // gerando (freio): calor desprezível no modelo
  float p = i * i * R;
  elev += (p - elev / MOTOR_R_TERM_KW) / MOTOR_C_TERM_JK * dt;
  return elev < 0 ? 0 : elev;
}

void cinematica_init() { tAnt = micros(); cinematica_para(); }

static float sinal(float x) { return x > 0 ? 1.0f : (x < 0 ? -1.0f : 0.0f); }

// Limitador do SpeedLimiter: o comando anda na direção do alvo no máximo "passo" por chamada.
static float limita(float alvo, float atual, float passo) {
  float d = alvo - atual;
  if (d >  passo) d =  passo;
  if (d < -passo) d = -passo;
  return atual + d;
}

// Desaturação com prioridade para o giro. A diferença de velocidade entre as rodas que o giro
// pede (w.B/2 em cada uma) é garantida primeiro, e o avanço fica com o que sobra até a
// velocidade máxima. Assim, com o oponente na diagonal, o robô sempre consegue virar para ele.
static void desatura(float& v, float& w) {
  float dif = fabsf(w) * BITOLA_MM / 2.0f;
  if (dif > VEL_MAX_MMS) { w = sinal(w) * VEL_MAX_MMS * 2.0f / BITOLA_MM; dif = VEL_MAX_MMS; }
  float vLim = VEL_MAX_MMS - dif;
  if (v >  vLim) v =  vLim;
  if (v < -vLim) v = -vLim;
}

// Feed-forward de uma roda: tensão para andar a v (mm/s) com aceleração a (mm/s²), em fração da
// tensão disponível no motor (-1 a 1). Abaixo de 2 mm/s a roda fica parada (freada).
static float fracao(float v, float a) {
  if (fabsf(v) < 2.0f) return 0;
  float u = (CIN_KS * sinal(v) + CIN_KV * v + CIN_KA * a) / CIN_V_DISP;
  return constrain(u, -1.0f, 1.0f);
}

// Modelo do motor rodado ao contrário: a fração aplicada leva a roda para a velocidade de
// regime (u.Vdisp - kS)/kV, com o atraso de primeira ordem MOTOR_TAU_MS.
static float estima(float roda, float u, float dt) {
  float vm = fabsf(u) * CIN_V_DISP;
  float alvo = vm > CIN_KS ? sinal(u) * (vm - CIN_KS) / CIN_KV : 0;
  float k = dt / (MOTOR_TAU_MS / 1000.0f + dt);
  return roda + (alvo - roda) * k;
}

void cinematica_comanda(float v_mms, float w_dps, Perfil perfil) {
  uint32_t agora = micros();
  float dt = (agora - tAnt) * 1e-6f;
  tAnt = agora;
  if (dt > 0.05f) dt = 0.05f;                 // primeira chamada ou loop que ficou parado
  if (dt < 1e-4f) dt = 1e-4f;

  float w = w_dps * (float)DEG_TO_RAD;        // a conta interna usa rad/s
  desatura(v_mms, w);

  float eAnt = vCmd - wCmd * BITOLA_MM / 2.0f, dAnt = vCmd + wCmd * BITOLA_MM / 2.0f;
  if (perfil == PERFIL_BORDA) {
    vCmd = v_mms; wCmd = w;
  } else {
    vCmd = limita(v_mms, vCmd, ACEL_MAX_MMS2 * dt);
    wCmd = limita(w, wCmd, ACEL_MAX_MMS2 * dt * 2.0f / BITOLA_MM);
  }
  // cinemática inversa (diff_drive_controller): cada roda anda v -/+ w.B/2
  float alvoE = vCmd - wCmd * BITOLA_MM / 2.0f;
  float alvoD = vCmd + wCmd * BITOLA_MM / 2.0f;

  float uE, uD;
  if (perfil == PERFIL_BORDA) {               // fuga da borda: tensão toda, sem esperar rampa
    // roda pedida abaixo de 2 mm/s fica parada, como no feed-forward: na SAIDA_LATERAL a roda de
    // dentro recebe v - w.B/2, que dá zero só no papel; o arredondamento do float deixava um
    // resto de -0,00001 mm/s, e o sinal disso mandava a roda inteira para trás
    uE = fabsf(alvoE) < 2.0f ? 0.0f : sinal(alvoE);
    uD = fabsf(alvoD) < 2.0f ? 0.0f : sinal(alvoD);
  } else {
    uE = fracao(alvoE, (alvoE - eAnt) / dt);
    uD = fracao(alvoD, (alvoD - dAnt) / dt);
    if (quente) {                             // proteção térmica: só nas manobras normais
      uE = constrain(uE, -MOTOR_PWM_QUENTE, MOTOR_PWM_QUENTE);
      uD = constrain(uD, -MOTOR_PWM_QUENTE, MOTOR_PWM_QUENTE);
    }
  }
  motores((int16_t)lroundf(uE * 255.0f), (int16_t)lroundf(uD * 255.0f));

  aquecE = aquece(aquecE, uE, rodaE, dt);
  aquecD = aquece(aquecD, uD, rodaD, dt);
  float a = aquecE > aquecD ? aquecE : aquecD;
  if (!quente && a >= MOTOR_QUENTE_K) quente = true;
  else if (quente && a <= MOTOR_FRIO_K) quente = false;
  rodaE = estima(rodaE, uE, dt);
  rodaD = estima(rodaD, uD, dt);
  giroAcum += cinematica_w_dps() * dt;
  distAcum += cinematica_v_mms() * dt;
}

void cinematica_para() { cinematica_comanda(0, 0, PERFIL_BORDA); }

float cinematica_v_mms()     { return (rodaE + rodaD) / 2.0f; }
float cinematica_w_dps()     { return (rodaD - rodaE) / BITOLA_MM * (float)RAD_TO_DEG; }
float cinematica_giro()      { return giroAcum; }
void  cinematica_rodas_travadas(bool t) { travadas = t; }
float cinematica_aquecimento_k()  { return aquecE > aquecD ? aquecE : aquecD; }
bool  cinematica_limite_termico() { return quente; }
float cinematica_dist()      { return distAcum; }
void  cinematica_zera_odometria() { giroAcum = 0; distAcum = 0; }
float cinematica_roda_esq()  { return rodaE; }
float cinematica_roda_dir()  { return rodaD; }
