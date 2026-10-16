// chave.h
// Chave DIP de 4 vias da placa (GPIOs 12, 14, 2 e 15): as vias 1 a 3 escolhem a estratégia e a
// via 4 escolhe o perfil de oponente escuro. A leitura passa por um filtro de 30 ms contra
// trepidação, e os comandos "estrategia" e "perfil" do terminal podem passar por cima da chave
// para testes na bancada.
#pragma once
#include <Arduino.h>

enum Estrategia : uint8_t { RUSH, FLANK, FAN_OUT, N_ESTRATEGIAS };

void        chave_init();
void        chave_atualiza();                // uma chamada por volta do loop
uint8_t     chave_bits();                    // bit 0 = DIP 1 ... bit 3 = DIP 4, já filtrados
Estrategia  chave_estrategia();              // escolha atual (chave ou comando)
bool        chave_escuro();                  // perfil de oponente escuro (chave ou comando)
bool        chave_conflito();                // mais de uma via de estratégia ligada
const char* estrategia_nome(Estrategia e);
// comandos do terminal: -1 volta a seguir a chave
void        chave_forca_estrategia(int8_t e);
void        chave_forca_escuro(int8_t escuro);
