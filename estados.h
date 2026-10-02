// estados.h
// Máquina de estados do robô. A estratégia lê somente a percepção P (percepcao.h) e
// manda comandos para os motores (motores.h); não existe acesso direto a sensor aqui.
#pragma once
#include <Arduino.h>

enum Estado : uint8_t {
  ESPERA,       // parado, aguardando o start
  CONTAGEM,     // 5 s parado, como manda a regra
  BUSCA,        // ninguém à vista: o robô segue o rumo (seta do painel) e de tempos em tempos avança um pouco
  MIRA,         // oponente visto fora do centro: correção do ângulo
  ATAQUE,       // oponente centrado: o robô empurra
  RECUO,        // borda na frente: o robô recua enquanto vê branco e mais uma folga
  GIRO_BORDA,   // giro para dentro do dohyo sem deixar de vigiar o oponente
  AVANCO_BORDA, // borda atrás (robô sendo empurrado): o robô avança enquanto vê branco e mais uma folga
  N_ESTADOS     // quantidade de estados, usada no tamanho do vetor de nomes
};

void        estados_init();
void        estados_passo();          // uma chamada por volta do loop, depois de percepcao_atualiza()
void        estados_iniciar();        // mesmo efeito do botão de start (comando "go")
void        estados_parar();          // retorno para ESPERA (comando "stop")
Estado      estado_atual();
const char* estado_nome(Estado e);
