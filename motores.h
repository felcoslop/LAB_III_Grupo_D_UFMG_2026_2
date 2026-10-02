// motores.h
// Interface dos motores. Cada roda recebe um valor de -255 a 255, e o positivo anda para a frente.
// Quem comanda os motores é a máquina de estados (estados.cpp); o oponente.cpp e o debug.cpp
// só leem o último comando.
#pragma once
#include <Arduino.h>

void motores_init();
void motores(int16_t esq, int16_t dir);
void motores_para();
int16_t motores_esq();   // último comando enviado, usado no debug e para estimar o giro no rastreador
int16_t motores_dir();
