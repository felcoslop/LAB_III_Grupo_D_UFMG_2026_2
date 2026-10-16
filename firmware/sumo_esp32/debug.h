// debug.h
// Comandos de teste e calibração digitados no Monitor Serial (115200, "Nova linha") ou
// enviados pelo painel. As funções são chamadas pelo sumo_esp32.ino.
#pragma once
#include <Arduino.h>

void debug_init();
void debug_passo();   // uma chamada por volta do loop; não bloqueia, com exceção do comando "medir"
