// oponente.h
// Interface do módulo dos sensores de oponente (VL53L0X). O oponente.cpp liga os sensores,
// lê as distâncias, aplica a calibração e entrega tudo para a fusão, que transforma as
// leituras (5 ou 7, conforme o QTD_TOF) em um único "sensor de oponente" dentro de P.op. Quem chama essas funções é o
// percepcao.cpp (leitura e fusão) e o debug.cpp (comandos de teste e calibração).

#pragma once
#include <Arduino.h>
#include "percepcao.h"

uint8_t  tof_init();                  // ligação dos sensores um por um pelo XSHUT e troca de endereço; o retorno é quantos responderam
                                      // (no fim, a tarefa dos sensores começa no núcleo SENSORES_NUCLEO)
uint8_t  tof_reinicia_falhos();       // nova tentativa nos que falharam (bloqueia uns 20 ms por sensor)
void     tof_atualiza();              // sem a tarefa do núcleo 0, faz a leitura e o passo do rastreador no loop
void     tof_libera_manutencao(bool sim);   // nova tentativa automática nos sensores que caíram (só parado)
void     tof_perfil(bool escuro);     // perfil do oponente pedido pela chave DIP 4 (aplicado pela tarefa)
bool     tof_perfil_escuro();         // perfil já aplicado nos sensores
void     oponente_borda_tras(bool branco);  // borda de trás no branco: os ToF de trás ignoram o que fica além dela
bool     oponente_borda_tras_recente();     // borda de trás vista há menos de T_TRAS_BRANCO_MS
void     tof_i2c_pega();              // reserva do I2C para comandos do terminal (scan, init)
void     tof_i2c_solta();
uint16_t tof_descartes();             // leituras descartadas por cair fora do dojô (contador)
// Sensores com leitura abaixo de TOF_MIN_MM nos últimos janela_ms (bit i = TOF[i]): algo
// encostado no robô daquele lado. O rastreador não usa essa leitura (fica abaixo do mínimo do
// VL53L0X), mas ela indica contato, e com contato o movimento das rodas não é confiável.
uint16_t tof_colados(uint16_t janela_ms);
// Sensores com leitura colada antes da luta (obstrução): o aviso aparece no comando "tof"
uint16_t tof_obstruidos();
bool     tof_ok(uint8_t i);           // true quando o sensor i está respondendo
uint16_t tof_mascara_ok();            // bit i ligado quando TOF[i] está OK
int16_t  tof_mm(uint8_t i);           // leitura corrigida e filtrada em mm (-1 quando não tem nada)
int16_t  tof_bruto(uint8_t i);        // leitura crua do chip (-1 quando não tem nada)
uint16_t tof_seq(uint8_t i);          // contador de medidas novas, usado nas médias da calibração
int8_t   tof_indice(const char* nome);// índice da tabela TOF para um nome como "FC" (-1 quando não existe)
int8_t   tof_centro();                // índice do sensor central (ângulo mais perto de 0)

// calibração de 2 pontos, gravada na memória do ESP32
void     tof_cal_define(uint8_t i, float ganho, float offset);
void     tof_cal_zera(uint8_t i);
float    tof_cal_ganho(uint8_t i);
float    tof_cal_offset(uint8_t i);
bool     tof_cal_gravada(uint8_t i);

void     oponente_funde(Oponente& op);// preenchimento de P.op a partir das leituras
bool     oponente_bayes();            // true quando o rastreador bayesiano está em uso (FUSAO_BAYES 1)
int16_t  tof_cru(uint8_t i);          // última medida corrigida sem mediana (-1 nada, -2 sem medida)
