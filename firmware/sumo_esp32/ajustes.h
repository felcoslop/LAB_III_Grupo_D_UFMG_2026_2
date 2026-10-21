// ajustes.h
// Opções que o painel muda sem recompilar o firmware (a struct Ajustes e o valor de fábrica ficam no config.h).
// Elas ficam gravadas na memória não volátil do ESP32 (NVS, namespace "ajustes") e são lidas no começo do setup,
// antes de qualquer módulo usar N_TOF ou a posição dos sensores de linha.
//   nome     valores   efeito
//   auto     0 ou 1    modo autônomo: contagem de 5 s e luta sozinho depois de ligar (AUTO_START); vale no próximo boot
//   linha    0 ou 1    0 sensores de linha da frente como no CAD, 1 colados de lado (BORDA_LATERAL_*); vale na hora
//   tof      5 ou 7    sensores de oponente em uso; o robô reinicia, porque os sensores são configurados no boot
//   alinha   0 ou 1    na contagem de 5 s só gira até ficar de frente para o oponente; BOOT reinicia a contagem
//   wifi     0 ou 1    o rádio continua ligado na luta, mesmo sem painel conectado
//   leve     0 ou 1    Wi-Fi leve: pelo rádio só o essencial (D e Q em taxa baixa, sem mapa); vale na hora, a USB não muda
// O comando do terminal e do painel é "ajuste NOME VALOR" (debug.cpp); "ajuste padrao" apaga tudo e volta ao valor de fábrica.
#pragma once
#include "config.h"

enum AjusteResultado : int8_t { AJUSTE_INVALIDO = -1, AJUSTE_OK = 0, AJUSTE_REINICIA = 1 };

void ajustes_carrega();                                // lê a NVS (chamar uma vez, no começo do setup)
int8_t ajustes_define(const char* nome, int valor);    // valida, aplica e grava; AJUSTE_REINICIA quando o robô precisa reiniciar
int8_t ajustes_padrao();                               // apaga os ajustes gravados e volta ao valor de fábrica (AJUSTE_REINICIA se a quantidade de ToF muda)
bool ajustes_gravado();                                // true quando existe algum ajuste na NVS (o robô não está no valor de fábrica)
