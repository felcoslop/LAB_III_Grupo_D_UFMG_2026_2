// estados.h
// Máquina de estados do robô. A estratégia lê somente a percepção P (percepcao.h) e
// manda comandos para a cinemática (cinematica.h); não existe acesso direto a sensor aqui.
#pragma once
#include <Arduino.h>
#include "chave.h"

enum Estado : uint8_t {
  ESPERA,        // parado, aguardando o start
  CONTAGEM,      // 5 s parado, como manda a regra
  BUSCA,         // ninguém à vista: volta ao centro do dojô e procura (giro ou leque, conforme a estratégia)
  MIRA,          // oponente visto fora do centro: correção do ângulo
  ATAQUE,        // oponente centrado: o robô empurra com o torque todo
  FLANCO,        // estratégia Flank: curva para chegar pela lateral do oponente
  ATAQUE_RE,     // oponente colado atrás: o robô empurra de ré, sem gastar tempo girando na frente dele
  DESTRAVA,      // giro preso pelo oponente encostado: o robô solta a quina e volta a mirar
  RECUO,         // borda na frente: o robô recua enquanto vê branco e mais uma folga
  GIRO_BORDA,    // giro para o centro do dojô sem deixar de vigiar o oponente
  AVANCO_BORDA,  // borda atrás (robô sendo empurrado): o robô avança enquanto vê branco e mais uma folga
  SAIDA_LATERAL, // borda em um lado inteiro: curva para dentro apoiada na roda de dentro
  N_ESTADOS      // quantidade de estados, usada no tamanho do vetor de nomes
};

void        estados_init();
void        estados_passo();          // uma chamada por volta do loop, depois de percepcao_atualiza()
void        estados_iniciar();        // mesmo efeito do botão de start (comando "go")
void        estados_parar();          // retorno para ESPERA (comando "stop")
void        estados_teste_mov(float v_mms, float w_dps, uint16_t ms);   // movimento de teste em ESPERA (comando "mov")
void        estados_teste_pwm(int8_t esq_pct, int8_t dir_pct, uint16_t ms); // PWM direto em cada roda (comando "pwm")
bool        estados_teste_ativo();    // true durante um teste mov ou pwm
Estado      estado_atual();
const char* estado_nome(Estado e);
Estrategia  estados_estrategia();     // estratégia em uso (travada no start)
bool        estados_escuro();         // perfil de oponente em uso (travado no start)
// Travamento: 0 livre, 1 empurrão em contato há mais de T_TRAVA_MS, 2 giro preso (DESTRAVA)
uint8_t     estados_travado();
uint16_t    estados_destravas();      // manobras DESTRAVA nesta luta
