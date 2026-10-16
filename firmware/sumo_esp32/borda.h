// borda.h
// Interface dos 4 sensores TCRT5000 que procuram a linha branca do dohyo, lidos pela saída
// analógica. O borda.cpp preenche a parte P.borda da percepção (chamado pelo percepcao.cpp),
// e as outras funções servem para o debug.cpp mostrar valores e fazer a calibração.
#pragma once
#include <Arduino.h>
#include "percepcao.h"

void     borda_init();
void     borda_atualiza(Borda& b);        // leitura dos 4 sensores, filtro e preenchimento de P.borda
uint16_t borda_mv(uint8_t i);             // última leitura do sensor i em mV
bool     borda_branco_cru(uint8_t i);     // branco nesta leitura, ainda sem a confirmação de tempo
uint16_t borda_limiar(uint8_t i);         // limiar em uso no sensor i (mV)
bool     borda_branco_abaixo(uint8_t i);  // true quando o branco lê uma tensão menor que o limiar

// calibração: média no preto e no branco, gravada na memória do ESP32
void     borda_cal_define(uint8_t i, uint16_t preto_mv, uint16_t branco_mv);
void     borda_cal_zera();
bool     borda_cal_gravada(uint8_t i);
uint16_t borda_cal_preto(uint8_t i);
uint16_t borda_cal_branco(uint8_t i);
