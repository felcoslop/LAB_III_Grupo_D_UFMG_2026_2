// cinematica.h
// Cinemática das duas rodas (robô diferencial) e modelo do acionamento. A máquina de estados
// pede um avanço v em mm/s e um giro w em graus/s (positivo é para a esquerda, o mesmo sentido
// dos ângulos dos sensores), e este módulo transforma esse pedido na velocidade de cada roda e
// no PWM de cada lado do L298N:
//
//   1. desaturação: se uma roda precisaria passar da velocidade máxima, o avanço diminui e o
//      giro fica, para a mira nunca ser perdida (prioridade para o giro);
//   2. limitador de aceleração nas manobras normais, para a roda não patinar (o perfil da borda
//      não tem limitador e usa a tensão toda na hora);
//   3. cinemática inversa: vE = v - w.B/2 e vD = v + w.B/2, com B a bitola;
//   4. feed-forward do motor: tensão = kS.sinal(v) + kV.v + kA.a em cada roda, dividida pela
//      tensão que chega no motor para virar PWM.
//
// O mesmo modelo, rodado ao contrário, estima a velocidade real de cada roda (com o atraso do
// motor), e essa estimativa vai para o rastreador e para o giro de recuperação na borda.
// Bases usadas: cinemática e limitador do diff_drive_controller do ros2_controllers, saturação
// das rodas do linorobot2_hardware e feed-forward SimpleMotorFeedforward do WPILib.
#pragma once
#include <Arduino.h>
#include "config.h"

// Constantes derivadas do config.h. O modelo do motor é uma reta tensão x velocidade que passa
// pelos dois pontos da tabela: 1,5 V para sair do lugar e 319 rpm livre a 12 V.
constexpr float CIN_RAIO_MM   = RODA_DIAMETRO_MM / 2.0f;
constexpr float CIN_W_LIVRE   = MOTOR_RPM_LIVRE * 2.0f * (float)PI / 60.0f;              // rad/s no eixo
constexpr float CIN_KS        = MOTOR_V_PARTIDA;                                          // V
constexpr float CIN_KV        = (MOTOR_V_NOMINAL - MOTOR_V_PARTIDA) / (CIN_W_LIVRE * CIN_RAIO_MM);   // V por mm/s
constexpr float CIN_KA        = MOTOR_KA / 1000.0f;                                       // V por mm/s²
constexpr float CIN_V_DISP    = BATERIA_V - PONTE_QUEDA_V;                                // V no motor com PWM 100%
constexpr float VEL_MAX_MMS   = (CIN_V_DISP - CIN_KS) / CIN_KV;                           // cerca de 410 mm/s
constexpr float GIRO_MAX_DPS  = 2.0f * VEL_MAX_MMS / BITOLA_MM * 180.0f / (float)PI;     // cerca de 360 graus/s

// Distância de parada na borda, no pior caso: bateria cheia, robô na velocidade máxima, tempo
// de reação (confirmação da borda e volta do loop) e roda patinando para trás na frenagem.
constexpr float CIN_VEL_CHEIA = (BATERIA_V_CHEIA - PONTE_QUEDA_V - CIN_KS) / CIN_KV;
constexpr float DIST_PARADA_MM = CIN_VEL_CHEIA * (BORDA_CONFIRMA_MS + 3) / 1000.0f
                               + CIN_VEL_CHEIA * CIN_VEL_CHEIA / (2.0f * FREIO_MIN_MMS2);

enum Perfil : uint8_t {
  PERFIL_NORMAL,   // manobras de luta: com limitador de aceleração e feed-forward
  PERFIL_BORDA     // fuga da borda: sem limitador, tensão toda no sentido pedido
};

void  cinematica_init();
void  cinematica_comanda(float v_mms, float w_dps, Perfil perfil = PERFIL_NORMAL);   // uma chamada por volta do loop
void  cinematica_para();          // freio nas duas rodas (os dois IN do L298N em LOW)
float cinematica_v_mms();         // avanço estimado do robô
float cinematica_w_dps();         // giro estimado do robô
float cinematica_giro();          // giro acumulado (graus) desde o último cinematica_zera_odometria()
float cinematica_dist();          // avanço acumulado (mm) desde o último cinematica_zera_odometria()
void  cinematica_zera_odometria();
float cinematica_roda_esq();      // velocidade estimada de cada roda (mm/s), para o debug
float cinematica_roda_dir();
