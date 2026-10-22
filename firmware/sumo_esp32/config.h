// config.h
// Todas as configurações que dependem do hardware ficam neste arquivo: pinos, posição dos
// sensores, limiares, velocidades e tempos. O resto do código só lê estes valores.
// As tabelas TOF[] e BORDA[] definem os sensores; o tamanho delas é calculado com sizeof,
// então um sensor a mais é só uma linha a mais na tabela.
//
// Identidade dos ToF: o ESP32 não sabe qual sensor está em qual lugar. Quem define o nome
// é o fio XSHUT, ou seja, o sensor ligado no pino XSHUT da linha "FC" passa a ser o FC.
// O comando "id" no terminal ajuda a conferir isso.

#pragma once
#include <Arduino.h>

// ============================================================================
// Barramento I2C, compartilhado por todos os ToF
// ============================================================================
#define PINO_SDA        21
#define PINO_SCL        22
#define I2C_CLOCK_HZ    100000   // 100 kHz é mais tolerante a fios longos de protoboard
                                 // na placa final, com fios curtos, o valor pode ir para 400000

// ============================================================================
// Sensores de oponente: VL53L0X (ToF), placas CJVL53L0XV2 / GY-530
// Referencial (o mesmo dos gabaritos em doc/pdf/): origem no centro de giro (na bancada, o
// centro da protoboard), x em mm para a frente, y em mm para a esquerda, ang em graus
// (0 é frente, positivo é esquerda). As coordenadas x,y são da lente de cada sensor.
// O offset_mm só vale enquanto o sensor não foi calibrado com o comando "cal"; a
// calibração de 2 pontos fica gravada na memória do ESP32 (NVS) e substitui esse valor.
// ============================================================================
struct CfgToF {
  const char* nome;
  uint8_t     xshut;      // pino do ESP32 ligado ao XSHUT deste sensor
  uint8_t     endereco;   // endereço I2C que o sensor recebe no boot
  int16_t     x, y, ang;
  int16_t     offset_mm;
};

// GEOMETRIA_BANCADA vale 0: o robô montado (STL do chassi) é a única geometria em uso. O valor 1 (bancada,
// protoboard 165 x 54,7 mm) sobrou só porque os geradores dos gabaritos em doc/pdf/ leem a tabela da bancada
// abaixo; não grave o robô com 1. O painel não mostra mais essa chave.
// O valor também pode vir da compilação (-DGEOMETRIA_BANCADA=0), como o simulador de luta faz.
#ifndef GEOMETRIA_BANCADA
#define GEOMETRIA_BANCADA 0
#endif

// Quantidade de sensores de oponente: 5 (montagem original) ou 7, com um par a mais atrás do eixo
// (TE e TD: a ±160 graus, olhando para trás, nas quinas da bancada e nas janelas das laterais do robô montado,
// com o suporte inclinado; a largura dele é pendência do chassi, veja PENDENTE NO CHASSI). A frente continua com os 3 sensores de antes e o FC segue
// sozinho no centro, decidindo o ataque, como pede o capítulo 2 do Sumo Robot Blackbook (número
// ímpar de sensores, com um no centro). Na simulação com o rastreador, o par de trás reduziu os
// ataques que chegam sem ser vistos de 36% para 8% e o tempo médio de busca de 206 para 35 ms,
// sem piorar a precisão na frente (detalhes no README e no guia de 7 sensores).
// O par novo fica no fim da tabela para os 5 sensores antigos manterem o mesmo endereço, cor no
// painel e calibração gravada.
// QTD_TOF é só o valor de fábrica: a quantidade em uso (N_TOF) é uma das opções do painel, gravada na memória
// do robô (comando "ajuste tof 5|7", ajustes.cpp), então trocar de 5 para 7 não pede regravar o firmware. As
// tabelas abaixo trazem sempre os sete sensores; com 5, os dois últimos (TE e TD) ficam fora de tudo e o XSHUT
// deles é mantido em LOW, para um sensor montado e desligado não atender no endereço padrão.
// O valor de fábrica também pode vir da compilação (-DQTD_TOF=7), como o CI faz.
// Com 7 sensores, o I2C a 100 kHz fica ocupado boa parte do tempo; na placa definitiva, com
// fios curtos, o I2C_CLOCK_HZ pode subir para 400000.
#ifndef QTD_TOF
#define QTD_TOF 5
#endif
#if QTD_TOF != 5 && QTD_TOF != 7
#error "QTD_TOF precisa ser 5 ou 7"
#endif

// Opções que o painel muda sem recompilar (ajustes.cpp grava na NVS e lê no boot). O valor de fábrica de cada
// uma vem das chaves deste arquivo (QTD_TOF, AUTO_START, WIFI_LEVE); as outras começam desligadas.
struct Ajustes {
  uint8_t tof;               // sensores de oponente em uso: 5 ou 7
  bool    autonomo;          // partida automática depois de ligar (AUTO_START)
  bool    borda_lateral;     // BFE e BFD colados de lado, 90 graus do encaixe do CAD (BORDA_LATERAL_*)
  bool    alinha_contagem;   // durante a contagem de 5 s o robô só gira até ficar de frente para o oponente
  bool    wifi_luta;         // o rádio fica ligado depois do start, mesmo sem painel conectado
  bool    wifi_leve;         // Wi-Fi leve: pelo rádio só o essencial (D, Q em taxa baixa, sem mapa); a USB não muda
};
inline Ajustes& ajustes();

#if GEOMETRIA_BANCADA
// Protoboard 165 x 54,7 x 10,2 mm com a linha 63 na frente; a origem é o centro dela.
// Medidas da montagem real (01/10): FE e FD a 54,4 mm um do outro, FC a 31,6 mm do FE e a
// 29,9 mm do FD. Com essas três distâncias o FC fica cerca de 14 mm à frente dos cantos e
// 1 mm para a direita. LE e LD ficam a 59,3 mm um do outro, no meio da protoboard.
// TE e TD (7 sensores) ainda não foram montados: a posição abaixo é a prevista no gabarito
// (doc/pdf/guia_montagem_sensores_7.pdf), nas quinas de trás da protoboard, com o palito colado por
// fora para não atrapalhar os fios das linhas 1 a 4. Depois de montar, a posição medida
// substitui estes números.
static const CfgToF TOF[] = {
  //  nome   xshut  endereço   x     y    ang   offset
  {  "LE",   17,    0x30,     -1,   30,   90,    0  },   // meio da lateral esquerda
  {  "FE",   23,    0x31,     73,   27,   20,    0  },   // canto frente esquerdo
  {  "FC",   19,    0x32,     87,   -1,    0,    0  },   // centro da frente, decide o ataque
  {  "FD",   18,    0x33,     73,  -27,  -20,    0  },   // canto frente direito
  {  "LD",    5,    0x34,     -2,  -30,  -90,    0  },   // meio da lateral direita
  // XSHUT conforme a pinagem da placa nova (Lab3-Liga). O GPIO 5 é pino de boot e precisa estar
  // em nível alto no boot, que é o nível do XSHUT com o sensor ligado (pull-up do módulo).
  {  "TE",   16,    0x35,    -80,   25,  160,    0  },   // quina traseira esquerda (prevista), só com 7 sensores
  {  "TD",    4,    0x36,    -80,  -25, -160,    0  },   // quina traseira direita (prevista), só com 7 sensores
};
// contorno do corpo em mm a partir da origem e altura das lentes, usados pelo painel
#define CORPO_FRENTE_MM   82.5f
#define CORPO_TRAS_MM     82.5f
#define CORPO_ESQ_MM      27.35f
#define CORPO_DIR_MM      27.35f
#define CORPO_ALTURA_MM   10.2f    // espessura da protoboard
#define LENTE_ALTURA_MM   23.0f    // centro das lentes, entre 21 e 24 mm acima da base
#define GEOMETRIA_NOME    "BANCADA"
#else
// Robô montado, com o chassi de dois andares do editor 3D v11 (STL do grupo, 10/2026; a página
// site/montagem.html confere esta tabela contra o editor). A origem fica no meio do eixo das rodas,
// que é o centro de giro, no chão, com x para a frente e y para a esquerda: a ponta da rampa fica
// 118 mm à frente dele, a traseira da base 34 mm atrás, e o corpo tem 151 mm de largura (os suportes
// de 160 graus do TE e do TD sobram 1 mm de cada lado da lateral; o gabarito da largada é 152 mm).
// Cada VL53L0X vai no suporte de PLA do grupo, com a lente virada para fora:
//   - FE, FC e FD ficam na face inclinada da frente (FE e FD a 17 graus para fora);
//   - LE e LD ficam na lateral à frente do eixo, retos, a 90 graus;
//   - TE e TD ficam atrás da roda, em suportes inclinados, e olham para trás a 160 graus, como no
//     estudo de 7 sensores (par atrás, ±160 graus).
//   PENDENTE NO CHASSI (doc/montagem/README.md, seção Espaço no chassi):
//   1. Inclinação: o suporte da face da frente deixa os feixes do FE, FC e FD uns 27 graus para
//      cima. Esta tabela e o firmware supõem feixe horizontal; com a inclinação, o cone passa por
//      cima de um oponente baixo a partir de uns 150 mm. Corrigir o suporte (calço ou janela) ou
//      aceitar o alcance menor.
//   2. A lente do TE e do TD fica a 29 a 30 mm do chão, 5 mm acima das outras (LENTE_ALTURA_MM é
//      a altura das lentes da frente e das laterais).
static const CfgToF TOF[] = {
  //  nome   xshut  endereço   x     y    ang   offset
  {  "LE",   17,    0x30,     46,    70,   90,    0  },   // lateral esquerda, à frente do eixo
  {  "FE",   23,    0x31,     77,    32,   17,    0  },   // face da frente, lado esquerdo
  {  "FC",   19,    0x32,     83,     0,    0,    0  },   // face da frente, meio, decide o ataque
  {  "FD",   18,    0x33,     77,   -34,  -17,    0  },   // face da frente, lado direito
  {  "LD",    5,    0x34,     46,   -70,  -90,    0  },   // lateral direita, à frente do eixo
  {  "TE",   16,    0x35,    -30,    66,  160,    0  },   // atrás da roda esquerda, para trás (só com 7 sensores)
  {  "TD",    4,    0x36,    -30,   -66, -160,    0  },   // atrás da roda direita, para trás (só com 7 sensores)
};
#define CORPO_FRENTE_MM   118.0f   // ponta da rampa
#define CORPO_TRAS_MM     34.0f    // face de trás da base
#define CORPO_ESQ_MM      75.5f
#define CORPO_DIR_MM      75.5f
#define CORPO_ALTURA_MM   55.0f    // topo da tampa da frente em relação ao chão (a de trás fica a 47 mm)
#define LENTE_ALTURA_MM   24.0f    // lentes da frente e das laterais; as do TE e do TD ficam a 29 e 30 mm
#define GEOMETRIA_NOME    "ROBO"
#endif
#define TOF_MAX (sizeof(TOF) / sizeof(TOF[0]))  // sensores da tabela (7); tamanho dos vetores por sensor
#define N_TOF   (ajustes().tof)                 // sensores em uso agora (5 ou 7), escolhidos no painel

#define TOF_TIMING_US     20000   // 20 ms por medida no perfil de oponente claro (o escuro usa 33 ms)
#define DOHYO_DIAMETRO_MM 770     // dojô da CoRA: 77 cm, a faixa branca faz parte dele
// Alcance contado a partir do centro do robô; leitura mais longe vira "nada". Pior caso dentro
// do dojô: o centro do robô chega a uns 340 mm do centro do dojô (sensor de borda na faixa, que
// começa a 360 mm, mais a distância de parada) e o oponente, inteiro dentro do dojô, tem o
// centro a até 385 - 76 = 309 mm do centro. Nos dois lados opostos, a face do oponente fica a
// 340 + 309 - 76 = 573 mm do centro do robô; com 27 mm de folga, o alcance fica em 600 mm. O que
// passa desse raio, ou cai fora do dojô pela localização (arena.cpp), não vira oponente.
#define TOF_ALCANCE_MM    600
// Os ToF laterais (|ângulo| de 80 a 120 graus: LE e LD) só valem
// até este alcance, a partir da face do sensor. No robô montado os dois sensores de cada lado
// olham para o mesmo lado ou quase (duas janelas por lateral), e a plateia em volta do dojô aparecia
// nos dois ao mesmo tempo: o rastreador somava as duas leituras e mirava uma pessoa por dezenas de
// segundos. Com 80 mm o lado só enxerga o oponente encostado ou quase (a frente, a 600 mm, cuida do
// resto). Na bateria com pessoas em volta do dojô (540 lutas) o maior tempo mirando uma pessoa foi
// de 49,7 s com 180 mm, 29,8 s com 150 mm, 5,9 s com 120 mm, 29,6 s com 100 mm e 1,0 s com 80 mm, e
// as vitórias ficaram entre 347 e 355 em todos. COMO AJUSTAR: aumente (100 a 150) se o robô perder o
// oponente que passa pela lateral a mais de 8 cm; diminua (50 a 60) se ainda mirar a plateia. Um
// valor alto (1000 ou mais) desliga o limite.
// TOF_ALCANCE_TRAS_MM vale para o par que olha para trás (TE e TD, |ângulo| de 120 graus ou mais, a
// 160 no robô montado). Na bateria de 540 lutas com pessoas em volta do dojô, o par de trás com o
// alcance inteiro de 600 mm mirou a plateia por até 53 s em 11 lutas; limitado a 80 mm (o mesmo dos
// laterais) ou a 200 mm, o maior tempo mirando uma pessoa foi de 0,5 s e de 0,7 s, com 350 e 352
// vitórias (351 com o par reto a 90 graus). COMO AJUSTAR: o mesmo dos laterais; com 7 sensores o
// robô enxerga por trás o oponente que chega a até esta distância do sensor.
// DISTÂNCIA DA PLATEIA AO DOJÔ: contada da borda de fora da faixa branca, com a plateia plana e de frente
// para o robô, nenhum ToF lê a plateia a mais de 48 cm do robô com a ponta da rampa na linha olhando para
// fora (limite de TOF_ALCANCE_MM, 600 mm em volta do eixo), 13 cm com a traseira na linha e o robô de
// costas para a plateia (TE e TD, com TOF_ALCANCE_TRAS_MM) e 21 cm com o robô de lado (FE ou FD, que
// apontam 17 graus para fora; TE 11 cm, LE e LD 8 cm). Mudar os alcances muda esses números: a conta
// está no README (seção "Distância da plateia ao dojô"). Pessoa de verdade, mais estreita que uma
// parede, é lida de menos longe. Com a localização do dojô válida, o firmware ainda ignora leitura
// mais de ARENA_MARGEM_MM além do raio do dojô; a distância vale na largada, antes de a localização
// convergir, e se a incerteza passar de ARENA_SIGMA_DESCARTE.
#define TOF_ALCANCE_LATERAL_MM 80
#define TOF_ALCANCE_TRAS_MM 200
#define TOF_MIN_MM        15      // leitura menor que isso é descartada
#define TOF_TIMEOUT_MS    150     // sem medida nova por esse tempo, a leitura é descartada
#define TOF_PERDIDO_MS    500     // sem responder por esse tempo, o sensor fica marcado como FALHOU
#define TOF_RETRY_MS      2000    // com o robô parado, nova tentativa de religar a cada 2 s
#define TOF_MINIMO_PARA_LUTAR 3   // com menos ToF funcionando o robô não inicia (segurança)

// Calibração de 2 pontos (comando "cal NOME"): distâncias padrão do alvo em mm, medidas
// da face do chip até o alvo
#define CAL_P1_MM         50
#define CAL_P2_MM         300

// Fusão (Blackbook, capítulos 2 e 10)
#define FUSAO_PERDE_MS    100     // sem confirmação de nenhum sensor por esse tempo, o alvo é perdido
#define CENTRO_DEG        7       // |ângulo| até esse valor, com o sensor central vendo, conta como centrado

// FUSAO_BAYES 1 liga o rastreador bayesiano em grade (rastreador.cpp): ele sabe que existe
// no máximo um oponente quadrado de OPONENTE_LADO_MM e usa também o que os sensores NÃO
// veem. FUSAO_BAYES 0 volta para a fusão simples (média ponderada dos sensores que veem).
#define FUSAO_BAYES       1
#define OPONENTE_LADO_MM  152     // regra RoboCore 1 kg: 15,2 x 15,2 cm
#define ALCANCE_PRETO_MM  550     // distância até onde o ToF vê um robô preto sem falhar
                                  // (medida afastando um objeto preto até a leitura começar a piscar)
#define OP_VEL_MAX_MMS    1500    // velocidade máxima considerada para o oponente (mm/s)
#define GIRO_DESCONHECIDO_DPS 200 // sem motores (bancada girada na mão): giro possível sem aviso (°/s)
                                  // com motores, o giro e o avanço vêm da cinemática (cinematica.cpp)
#define FUSAO_MASSA_MIN   0.25f   // concentração mínima do mapa (0 a 1) para contar como "visto"
#define FUSAO_EXISTE_MIN  0.5f    // chance mínima de existir oponente para contar como "visto"
                                  // (arena vazia e leitura falsa isolada ficam abaixo disso)
#define MAPA_MS           250     // intervalo de envio do mapa de probabilidade para o painel
// Divisão entre os dois núcleos do ESP32: a leitura dos ToF pelo I2C e o rastreador rodam numa
// tarefa do núcleo 0, e o loop do Arduino, no núcleo 1, fica só com a borda, a localização, a
// máquina de estados, a cinemática, os motores e a USB. Assim a borda é lida a cada poucos
// microssegundos, sem esperar o I2C. Com -1 tudo roda dentro do loop, como antes.
#define SENSORES_NUCLEO   0

// Valores usados apenas quando FUSAO_BAYES é 0
#define FUSAO_CLUSTER_MM  150     // leituras com diferença até esse valor são o mesmo objeto
#define FUSAO_CLUSTER_DEG 45      // e só entre sensores vizinhos, com até 45° entre eles

// ============================================================================
// Sensores de borda: 4 módulos TCRT5000 (com LM393, pinos VCC GND D0 A0)
// O código usa a saída analógica A0 (Blackbook, capítulo 9), porque o D0 do comparador
// não distingue arranhão de linha; o trimpot azul só mexe no D0. Os pinos 34, 35, 32 e 33
// são entradas do ADC1. No esquema da placa nova o fio sai do D0: para a leitura analógica
// funcionar, o fio precisa sair do A0 do módulo. A alimentação é de 3,3 V, assim o A0 nunca passa de 3,3 V. Dohyo: 77 cm, laminado preto e borda branca de
// 2,5 cm. Altura boa do sensor ao chão: entre 3 e 8 mm.
// ============================================================================
#define USAR_BORDA        1

struct CfgBorda {
  const char* nome;
  uint8_t     pino;       // GPIO ligado ao A0 do módulo (pino S da placa de expansão)
  int16_t     x, y;       // posição em mm no mesmo referencial dos ToF, só para o painel
  int8_t      lado;       // -1 esquerda, +1 direita
  bool        frente;     // true na frente, false atrás
};

#if GEOMETRIA_BANCADA
// Sensores colados por fora das laterais da protoboard. Medidas (01/10): 68,3 mm entre os
// da frente e 68,2 mm entre os de trás, a cerca de 28/42 mm (frente) e 142/157 mm (trás)
// do início da protoboard e de 2,4 a 3 mm do chão.
static const CfgBorda BORDA[] = {
  //  nome    pino    x     y    lado  frente
  {  "BFE",   34,     40,   34,  -1,   true  },   // frente esquerda
  {  "BFD",   35,     55,  -34,  +1,   true  },   // frente direita
  {  "BTE",   32,    -75,   34,  -1,   false },   // trás esquerda
  {  "BTD",   33,    -60,  -34,  +1,   false },   // trás direita
};
#define BORDA_ALTURA_MM   2.7f     // altura da face do sensor ao chão, só para o painel 3D
#define BORDA_ATE_RODA_MM 40.0f    // menor distância em x entre um sensor de borda e o centro (rodas)
#else
// Os quatro TCRT5000 do chassi de dois andares, com a face do par infravermelho a 3 mm do chão e a
// placa para dentro do chassi: o par da frente fica nos furos da base a 95,5 mm do eixo, bem na
// frente dos motores e dos sensores laterais; o par de trás fica atrás da roda, a 26,5 mm do eixo
// e a 69 mm do centro, nas pontas da base. O static_assert da cinemática confere que a distância de
// parada cabe na margem entre o sensor de trás e o eixo (BORDA_ATE_RODA_MM).
static const CfgBorda BORDA[] = {
  {  "BFE",   34,     96,   62,  -1,   true  },
  {  "BFD",   35,     96,  -62,  +1,   true  },
  {  "BTE",   32,    -27,   69,  -1,   false },
  {  "BTD",   33,    -27,  -69,  +1,   false },
};
#define BORDA_ALTURA_MM   3.0f
#define BORDA_ATE_RODA_MM 26.5f    // menor distância em x entre um sensor de borda e o eixo das rodas
#endif
#define N_BORDA  (sizeof(BORDA) / sizeof(BORDA[0]))

// Sensores de linha da frente (BFE e BFD) colados de lado: provisoriamente o grupo colou os dois com cola quente
// virados para as laterais, 90 graus anti-horário do encaixe do CAD (o par infravermelho, que no CAD fica na ponta
// da frente da placa, passa para a ponta de fora). O painel escolhe entre as duas posições (comando "ajuste linha
// 0|1"). A posição abaixo é uma estimativa: placa de 38 mm girada 90 graus em torno do centro dela, com o par
// infravermelho a 19 mm do centro, e vale para os dois lados (o y muda de sinal). Só a localização no dojô (arena.cpp)
// e o desenho do painel usam esses números; a fuga da borda continua decidida por lado e frente. Meça a posição
// colada e corrija aqui se a diferença passar de uns 5 mm.
#define BORDA_LATERAL_X_MM  77
#define BORDA_LATERAL_Y_MM  81
inline int16_t borda_x(uint8_t i) {
  return (ajustes().borda_lateral && BORDA[i].frente) ? (int16_t)BORDA_LATERAL_X_MM : BORDA[i].x;
}
inline int16_t borda_y(uint8_t i) {
  return (ajustes().borda_lateral && BORDA[i].frente) ? (int16_t)(-BORDA[i].lado * BORDA_LATERAL_Y_MM) : BORDA[i].y;
}

// Limiar fixo usado enquanto não existe calibração ("calborda"). No TCRT5000 o branco lê
// tensão baixa. Medida na bancada (01/10, sensores a 3 mm do chão, 3,3 V): preto entre 1070
// e 1530 mV e branco perto de 260 mV. O valor antigo de 1500 mV caía dentro do preto.
#define BORDA_LIMIAR_MV     600
#define BORDA_BRANCO_ABAIXO 1
// Com calibração: limiar = branco + FRACAO x (preto - branco). O limiar fica perto do
// branco (0,35) para arranhão e poeira, que dão valores intermediários, não dispararem.
#define BORDA_FRACAO        0.35f
#define BORDA_CONFIRMA_MS   3      // branco contínuo por 3 ms e pelo menos 2 leituras é linha de verdade
                                   // (a linha de 2,5 cm passa em 25 ms com o robô a 1 m/s)

// ============================================================================
// Motores: módulo HW-956 (v0.0.2), ponte H dupla de MOSFET de 10 A por canal (15 A de pico,
// 3 a 18 V), com as entradas GND, IN1, IN2, IN3 e IN4. O esquema da placa mostra um L298N, mas
// o módulo usado é este. O anúncio do módulo aceita sinal de 3,3 V, limita o PWM a 2 kHz e pede
// parada antes de inverter o sentido, por causa do pico de corrente. Com o JGB37-520 esse pico,
// na inversão em velocidade máxima, fica perto de 2,4 A por motor (12,4 V da bateria somados a
// uns 11,4 V gerados pelo motor, sobre os cerca de 10 ohms do enrolamento), bem abaixo dos 10 A
// do módulo. Por isso a fuga da borda inverte na hora, sem pausa.
// O PWM vai no próprio pino IN: para a frente o IN1 recebe o PWM e o IN2 fica em LOW, e para
// trás o contrário. O módulo aceita PWM de até 2 kHz. Com os dois IN em LOW o motor para.
// Na bancada USAR_MOTORES fica em 0.
// O valor também pode vir da compilação (-DUSAR_MOTORES=1), como o CI faz.
// ============================================================================
#ifndef USAR_MOTORES
#define USAR_MOTORES      0
#endif
#define PINO_IN1          25     // motor esquerdo (saída do motor A do módulo)
#define PINO_IN2          26
#define PINO_IN3          27     // motor direito (saída do motor B)
#define PINO_IN4          13
#define INVERTE_ESQ       0      // 1 quando o motor esquerdo gira ao contrário
#define INVERTE_DIR       0
#define TROCA_MOTORES     0      // 1 quando o motor da esquerda está ligado na saída do motor B
#define PWM_MOTOR_HZ      1000   // frequência do PWM nos pinos IN (o módulo aceita até 2000)

// ============================================================================
// Cinemática das rodas e modelo do acionamento (cinematica.cpp)
// Motor JGB37-520 de 12 V e 319 rpm, redução de 18,8:1. Tabela do fabricante para essa linha:
// 0,1 A livre, 1,2 A e 1,9 kgf.cm com o eixo travado e partida com 1,5 V. O anúncio fala em
// 21 kgf.cm, valor que não fecha com 1,2 A nem com a redução de 18,8:1 e por isso fica de fora.
// Roda com o pneu StickyMAX S20 de 32 mm. Bateria 3S de Li-ion: 11,1 V nominal e 12,6 V cheia.
// A ponte H de MOSFET perde só alguns décimos de volt; um L298N perderia de 1,8 a 3,2 V com 1 A.
// Os valores marcados com "medir" ficam certos só com o robô montado: o Blackbook (capítulo 11)
// manda medir a distância percorrida em um tempo fixo e o tempo de uma volta completa no eixo.
// ============================================================================
#define MOTOR_V_NOMINAL   12.0f    // V
#define MOTOR_RPM_LIVRE   319.0f   // rpm no eixo de saída a 12 V, sem carga
#define MOTOR_V_PARTIDA   1.5f     // V, menor tensão que tira o motor do lugar (kS do feed-forward)
#define MOTOR_KA          1.25f    // V por m/s² em cada roda (estimado pela inércia do rotor; medir)
#define MOTOR_TAU_MS      60.0f    // atraso de cada roda com o robô em cima (estimado; medir)
#define BATERIA_V         11.1f    // V, usado para transformar tensão em PWM
#define BATERIA_V_CHEIA   12.6f    // V, usado só na conta da distância de parada
#define PONTE_QUEDA_V     0.2f     // V perdidos na ponte H de MOSFET (estimado; medir)
#define RODA_DIAMETRO_MM  32.0f
// distância entre os centros dos pneus, medida no STEP LAB3-Modelagem (10/2026); conferir no chassi
#define BITOLA_MM         126.8f
#define ACEL_MAX_MMS2     6000.0f  // limite das manobras normais, abaixo da tração (atrito ~1)
#define FREIO_MIN_MMS2    4400.0f  // pior frenagem: roda patinando, atrito 0,7 e 80% do peso nas rodas
#define BORDA_FAIXA_MM    25.0f    // largura da faixa branca da borda (regra RoboCore)

// ============================================================================
// Interface. A telemetria sai pela USB, para o Monitor Serial e para o painel (painel_sumo.html),
// e também pelo WiFi quando USAR_WIFI vale 1 (mais abaixo). O firmware nunca liga Bluetooth.
// ============================================================================
// Botão de start no GPIO 0, o mesmo do botão BOOT da placa, que já tem pull-up. O S1 do
// esquema fica entre o GPIO 0 e o GND, em paralelo com o BOOT. Com o botão apertado na hora de
// ligar, o ESP32 entra no modo de gravação, então o botão só pode ser usado com o robô ligado.
#define PINO_START        0      // nível baixo indica botão apertado (INPUT_PULLUP)

// Partida automática, para o robô sem botão de início. Com AUTO_START 1, depois de AUTO_START_ESPERA_MS com
// os sensores de oponente prontos (pelo menos TOF_MINIMO_PARA_LUTAR funcionando) e o perfil do oponente já
// aplicado, a ESPERA pede o start sozinha e o robô faz a contagem de T_CONTAGEM_MS e luta, como se o botão
// tivesse sido apertado. Vale uma vez por ligada: o comando "stop" volta para a ESPERA e o robô fica lá, e o
// botão BOOT e o comando "go" continuam valendo. Sem a chave DIP ligada, a estratégia é Rush e o oponente é
// claro. O padrão é 0, para o robô não andar sozinho quando liga na bancada ou na USB (com os motores
// ativos); para a competição, ponha 1 (ou compile com -DAUTO_START=1). Um fio do GPIO 0 ao GND NÃO serve
// de start: com o GPIO 0 em LOW na hora de ligar, o ESP32 entra no modo de gravação.
#ifndef AUTO_START
#define AUTO_START        0
#endif
#define AUTO_START_ESPERA_MS 1000   // sensores prontos há esse tempo antes de pedir o start
// AUTO_START é o valor de fábrica do "modo autônomo" do painel (comando "ajuste auto 0|1", gravado na NVS): o painel liga e
// desliga sem regravar o firmware, e a escolha vale a partir da próxima vez que o robô ligar (não larga na hora do clique).
// Os outros ajustes do painel (menos o wifi_leve, acima) começam desligados:
//   borda_lateral    BFE e BFD colados de lado, no lugar do encaixe do CAD (posição BORDA_LATERAL_* abaixo)
//   alinha_contagem  na contagem de 5 s o robô só gira em pulsos até ficar de frente para o oponente, e o botão BOOT
//                    reinicia a contagem (também no meio da luta, com o modo autônomo ligado). ATENÇÃO: o regulamento
//                    da CoRA manda o robô ficar parado nos 5 s; use na demonstração e desmarque na competição.
//   wifi_luta        o rádio continua ligado depois do start sem painel conectado, para acompanhar a luta pelo Wi-Fi
//                    (o regulamento proíbe rádio durante a luta; só para demonstração e testes)
// WIFI_LEVE é o valor de fábrica do "Wi-Fi leve" (comando "ajuste leve 0|1"). Pelo rádio, a telemetria completa leva uns
// 10 a 13 KB/s (D, Q e A a 50 Hz, mais o mapa M a 4 Hz) e disputa os buffers do lwIP e o ar com cada comando do /cmd; no robô
// isso deixou o controle remoto e os ajustes lentos. Com o modo leve, só para o rádio (a USB segue completa): a linha D sai
// a cada WIFI_LEVE_D_MS, a Q e a A a cada WIFI_LEVE_QA_MS e o mapa de probabilidade M a cada WIFI_LEVE_M_MS (0 tira o mapa); os
// textos, a Z do alinhamento, as linhas do cabeçalho (G, I, C, R, B, E, K, X, F) e a J da calibração da pista (que só existe
// enquanto o assistente roda) passam como antes. As leituras de todos os sensores seguem em tempo real (10 Hz) e o mapa a 2 Hz
// (uns 1,6 KB/s a mais, uns 3,6 KB/s no total); ele é uma linha de dados, então é o primeiro a ser descartado quando a rede
// enche e não atrasa o controle remoto (WASD), os ajustes nem a demonstração de alinhamento.
#ifndef WIFI_LEVE
#define WIFI_LEVE 1
#endif
#define WIFI_LEVE_D_MS 100
#define WIFI_LEVE_QA_MS 250
#define WIFI_LEVE_M_MS  500   // 0 = sem o mapa de probabilidade no modo leve
inline Ajustes& ajustes() {
  static Ajustes a = { QTD_TOF, AUTO_START != 0, false, false, false, WIFI_LEVE != 0 };
  return a;
}
// Demonstração de alinhamento (comando "alinha on", botão "Alinhar com o oponente" do painel). Com o robô parado em
// ESPERA ele só gira no eixo, em pulsos curtos separados por pausas em que os ToF medem sem o robô girando, até o
// oponente ficar a no máximo ALINHA_TOL_DEG do centro, e então fica parado e alinhado (a linha Z do painel mostra
// ALINHADO). Não anda para a frente nem para trás e a linha (borda) é ignorada: em ESPERA ela só encerra os testes
// mov e pwm. Como a linha está ignorada, o robô NÃO pode sair do lugar: a demonstração só acompanha o oponente que os
// ToF da frente enxergam (e, por ALINHA_SEGUE_MS depois de perdê-lo, a estimativa do rastreador). Sem oponente na
// frente ela fica parada, sem pulsos de busca às cegas, e o giro total desde o rumo em que o robô foi colocado e
// ligado nunca passa de ALINHA_GIRO_MAX_DEG para cada lado (0 tira o limite). Termina sozinho depois do tempo pedido,
// ao sair da ESPERA ou com "alinha off", e funciona com AUTO_START 0 ou 1 (com AUTO_START 1 o start automático fica
// suspenso enquanto o alinhamento roda). A busca em pulsos (ALINHA_BUSCA_*) fica só na contagem de 5 s com a opção
// "alinha" do painel (ajustes().alinha_contagem), que nunca anda e que o usuário escolhe de propósito.
#define ALINHA_TOL_DEG        3.0f   // alinhado: oponente visto a no máximo isso do centro do robô
#define ALINHA_SEGURA_MS      250    // dentro da tolerância por esse tempo (as pausas contam) para valer como alinhado
#define ALINHA_SAI_FATOR      1.7f   // alinhado só se desfaz com erro acima de ALINHA_TOL_DEG vezes isso (histerese)
#define ALINHA_PERDE_MS       600    // alinhado, sem ver o oponente por esse tempo: volta a procurar
#define ALINHA_K_DPS_GRAU     5.0f   // giro do pulso de acerto: graus/s para cada grau de erro
#define ALINHA_W_MIN_DPS      70     // giro mínimo do pulso (abaixo disso as rodas não vencem o atrito)
#define ALINHA_W_MAX_DPS      180    // giro máximo do pulso de acerto
#define ALINHA_FRACAO_PULSO   0.7f   // cada pulso gira essa fração do erro (o resto fica para o próximo, sem passar do ponto)
#define ALINHA_PULSO_MIN_MS   40
#define ALINHA_PULSO_MAX_MS   220
#define ALINHA_PAUSA_MS       160    // parado entre pulsos, para os ToF medirem
#define ALINHA_GIRO_MAX_DEG   90.0f  // demonstração: giro total máximo para cada lado desde o rumo inicial (0 = sem limite)
// Demonstração: só os ToF da frente contam (|ângulo do sensor| até ALINHA_SENSOR_FRENTE_DEG: FE, FC e FD). Com o oponente visto
// só por um lateral (LE, LD) ou por um traseiro (TE, TD), ou estimado a mais de ALINHA_FRENTE_MAX_DEG do centro, o robô fica
// parado. ALINHA_SO_FRENTE 0 volta a usar todos os sensores. A luta e a contagem de 5 s usam sempre todos.
#define ALINHA_SO_FRENTE      1
#define ALINHA_SENSOR_FRENTE_DEG 30   // sensor da frente: ângulo de montagem até isso (FE e FD a 17 graus, FC a 0)
#define ALINHA_FRENTE_MAX_DEG 35.0f  // campo dos três ToF da frente (17 graus + meio cone de 12,5) com folga
#define ALINHA_SEGUE_MS       500    // demonstração: depois de perder o oponente de vista, ainda segue a estimativa por esse tempo
#define ALINHA_BUSCA_W_DPS    130    // giro do pulso de busca (contagem de 5 s: oponente fora da vista)
#define ALINHA_BUSCA_PULSO_MS 230    // 30 graus por pulso, menos que o campo dos três ToF da frente
#define ALINHA_TEMPO_PADRAO_S 60     // "alinha on" sem tempo
#define ALINHA_TEMPO_MAX_S    600
// A chave DIP de 4 vias usa os GPIOs 12, 14, 2 e 15 e liga cada pino no 3V3. Por isso o GPIO 2
// não serve mais para o LED: com a chave ligada e o pino em LOW, o 3V3 ficaria em curto com o
// pino. O valor -1 desliga o LED de status. Cuidados no boot: DIP 1 (GPIO 12) desligada na hora
// de ligar o ESP32, porque nível alto no GPIO 12 muda a tensão da flash e a placa não liga, e
// DIP 3 (GPIO 2) desligada na hora de gravar o firmware.
#define PINO_LED          -1
// LED de robô travado (opcional, pode comprar depois): acende enquanto o empurrão estiver travado
// ou a manobra de destravar rodar, e apaga quando o robô se solta. Anodo no pino (resistor de
// 330 ohms em série) e catodo no GND. Com 7 ToF não sobra pino de saída livre: o GPIO 1 (TX0) é o
// único que serve, ao custo do terminal serial durante a luta (use a telemetria sem fio). Com 5
// ToF os GPIO 16 e 4 ficam livres. -1 deixa o LED desligado e o código nem entra no binário.
#ifndef PINO_LED_TRAVA
#define PINO_LED_TRAVA    -1
#endif
#define SERIAL_BAUD       115200
#define STREAM_MS         20     // dados para o painel a 50 Hz, o mesmo ritmo dos sensores

// WiFi opcional, para o painel funcionar sem cabo (demonstração, calibração da linha e controle
// remoto com o robô solto no dojô). USAR_WIFI 0 (padrão) NÃO compila nenhuma linha do rádio: o
// firmware fica idêntico ao de antes, com a RAM, a flash e o núcleo 0 inteiros para o robô. Com
// USAR_WIFI 1 o ESP32 cria a rede WIFI_AP_SSID (ou entra no roteador, se WIFI_STA_SSID estiver
// preenchido) e atende, na porta 80, /stream (dados em tempo real), /cmd?c=comando e /ping. O
// painel (painel_sumo.html) escolhe "Wi-Fi" na barra de conexão e usa o endereço 192.168.4.1.
// O custo do rádio é a pilha do WiFi disputando o núcleo 0 com o rastreador (foi o motivo da
// remoção em 78d843d): para medir no seu robô, rode o comando "tempo" com USAR_WIFI 0 e com 1 e
// compare o pior ciclo e o passo do rastreador. WIFI_DESLIGA_SEM_PAINEL 1 desliga o rádio sozinho
// na hora em que a luta começa (saída da ESPERA) sem nenhum painel conectado, então o robô de
// competição pode até sair com USAR_WIFI 1 sem pagar o rádio durante a luta. O comando "wifi off"
// desliga na hora e "wifi" mostra a rede, o endereço e quantos painéis estão conectados.
#ifndef USAR_WIFI
#define USAR_WIFI         0
#endif
#define WIFI_AP_SSID      "SUMO-ROBO"
#define WIFI_AP_SENHA     "sumo1234"     // no mínimo 8 caracteres
#define WIFI_AP_CANAL     6
#define WIFI_STA_SSID     ""             // nome do roteador (vazio usa só a rede própria)
#define WIFI_STA_SENHA    ""
#define WIFI_NOME_MDNS    "sumo"         // endereço http://sumo.local quando o sistema suporta
#define WIFI_DESLIGA_SEM_PAINEL 1        // 1: a luta sem painel conectado desliga o rádio

// ============================================================================
// Estratégia: avanço em mm/s, giro em graus/s (positivo é para a esquerda) e tempos em ms.
// A cinemática transforma cada par (avanço, giro) na velocidade e no PWM de cada roda. O
// ataque e a fuga da borda usam a velocidade máxima (VEL_MAX_MMS, calculada no cinematica.h).
// ============================================================================
#define T_CONTAGEM_MS     5000   // regra RoboCore: 5 s depois do comando de início
#define VEL_BUSCA_MMS     240    // avanço curto da BUSCA, cerca de metade da velocidade máxima
#define GIRO_BUSCA_DPS    200    // giro da BUSCA, cerca de metade do giro máximo
#define K_MIRA            6.0f   // mira: graus/s de giro para cada grau de erro (erro cai a 37% em ~0,17 s)
// Ataque: o mesmo ganho, só que menor. Com o oponente centrado, o giro de correção tira avanço
// das rodas (a cinemática dá prioridade ao giro), e o ataque chega mais devagar: a bateria de 1000
// lutas de frente, só com o Rush, deu 704 vitórias com 6, 715 com 3 e com 2, 719 com 1 e 721 com 0
// (derrotas de 1 a 5 em todas), e com 2 melhorou de 8 a 26 lutas e piorou de 3 a 7 em cada condição
// de robustez (rumo desalinhado em 8 e 15 graus, atrito 0,8 e 1,2). Ver README, "Otimização do ataque".
#define K_ATAQUE          2.0f   // ataque: graus/s de giro para cada grau de erro do oponente (a mira usa K_MIRA)
#define ANG_MIRA_CURVA    40     // até esse ângulo a mira é uma curva; acima, giro no eixo
#define GIRO_MIRA_MAX_DPS 370    // giro máximo na mira, 90% do giro máximo do robô
#define GIRO_BORDA_DPS    370    // giro de recuperação depois da borda
#define ANG_GIRO_BORDA    135    // tamanho desse giro em graus, estimado pela cinemática
#define T_GIRO_BORDA_MAX_MS 900  // segurança: o giro termina mesmo se a estimativa não chegar lá
#define T_RECUO_EXTRA_MS  10     // folga depois que a borda some (Blackbook, capítulo 10)
#define T_RECUO_BLOQ_MS   300    // recuo que não sai do branco nesse tempo vira curva de ré
#define T_BUSCA_GIRO_MS   1500   // BUSCA: tempo girando nos pontos cegos sem achar nada
#define T_BUSCA_AVANCO_MS 250    // BUSCA: tempo do avanço curto que muda o ponto de vista
// Vigia com o dojô vazio (oponente caiu ou não existe). Na BUSCA, depois de VIGIA_SEM_VER_MS sem ver o oponente, de
// uma volta inteira de giro desde a última vez que ele foi visto (VIGIA_VARREDURA_DEG) e com o rastreador achando que
// não há oponente no alcance (existe abaixo de VIGIA_EXISTE_ENTRA, sem rumo provável), o robô para no lugar: nada de
// avanço curto nem de volta ao centro. Só o cone da frente se mexe: o robô varre um setor em volta do rumo em que a
// vigia começou, em pulsos curtos de VIGIA_PULSO_DEG no eixo separados por VIGIA_PAUSA_MS parado (os ToF medem sem
// o robô girando): VIGIA_LADO_PULSOS pulsos para a esquerda, depois o dobro para a direita, depois o dobro para a
// esquerda, e assim por diante (3, 6, 6, 6... com os valores abaixo, ou seja, de +45 a -45 graus). O fim de cada lado é
// conferido também pelo giro estimado, para o setor não escorregar. Sai da vigia na hora em que algum sensor vê o
// oponente ou o rastreador volta a achar que ele existe (VIGIA_EXISTE_SAI, histerese). A borda continua com
// prioridade. Com oponente no dojô a vigia quase nunca entra (ver a bateria no README). VIGIA_DOJO_VAZIO 0 desliga.
#define VIGIA_DOJO_VAZIO     1
#define VIGIA_SEM_VER_MS     4000   // sem ver o oponente por esse tempo (contado desde a largada ou desde a última vez visto)
#define VIGIA_VARREDURA_DEG  360    // e girando pelo menos isso nesse tempo
#define VIGIA_EXISTE_ENTRA   0.15f  // chance de existir oponente abaixo disso para entrar
#define VIGIA_EXISTE_SAI     0.30f  // e acima disso para sair
#define VIGIA_PAUSA_MS       600    // parado entre os pulsos
#define VIGIA_PULSO_DEG      15     // tamanho de cada pulso
#define VIGIA_LADO_PULSOS    3      // pulsos do centro até cada ponta do setor (3 x 15 = 45 graus para cada lado)
#define VIGIA_GIRO_DPS       120    // velocidade do pulso (devagar: a roda não patina e o robô não sai do lugar)
#define VIGIA_PULSO_MAX_MS   500    // segurança: o pulso acaba nesse tempo mesmo se a estimativa não chegar ao ângulo
#define MODO_DEFENSIVO    0      // 1 faz o robô só mirar e nunca avançar

// ============================================================================
// Chave DIP de 4 vias: estratégia (DIP 1 a 3) e cor do oponente (DIP 4)
// Cada via liga o pino no 3V3, e o pull-down interno deixa o pino em LOW com a via desligada.
// DIP 1 = Rush, DIP 2 = Flank, DIP 3 = Fan out; nenhuma via ligada também vale Rush, então a
// DIP 1 pode ficar sempre desligada (o GPIO 12 em nível alto na hora de ligar muda a tensão da
// flash e o ESP32 não liga). Mais de uma via de estratégia ligada é um erro de ajuste: vale a de
// menor número e o terminal mostra um aviso. DIP 4 ligada escolhe o perfil de oponente escuro.
// A escolha fica travada no start: mexer na chave durante a luta não muda nada.
// ============================================================================
#define PINO_DIP1         12
#define PINO_DIP2         14
#define PINO_DIP3         2
#define PINO_DIP4         15
#define DIP_DEBOUNCE_MS   30

// ============================================================================
// Perfis de oponente (DIP 4)
// Claro: os valores de sempre, ajustados na bancada para oponente claro (TOF_TIMING_US,
// ALCANCE_PRETO_MM e o modelo do rastreador ficam como estavam).
// Escuro (preto fosco): o VL53L0X vai para o modo de longo alcance da Pololu e da ST, com
// limite de sinal de 0,10 MCPS (o padrão é 0,25), pulsos do laser mais longos (18 e 14 PCLK) e
// 33 ms por medida. O chip passa a aceitar retornos mais fracos e enxerga o preto mais longe, em
// troca de mais ruído e de mais leitura falsa. O rastreador acompanha a troca: espera um alvo
// que some com mais frequência (chance de detecção menor e queda mais suave com a distância),
// conta mais leitura falsa e segura o oponente por mais tempo antes de dar como perdido.
//
// COMO AJUSTAR (um valor de cada vez, testando com o oponente de cada cor no dojô):
//   ESCURO_SINAL_MCPS   menor (0,05) = o chip aceita retorno mais fraco: vê o preto mais longe, mas
//                       passa a ver poeira, reflexo do piso e a parede. Maior (0,20) = menos leitura
//                       falsa, mas o preto some a distâncias menores. Não passe de 0,25 (é o claro).
//   ESCURO_TIMING_US    maior (40000) = medida mais limpa e alcance maior, mas o robô reage mais
//                       devagar (cada leitura leva esse tempo). Menor (25000) = reação mais rápida e
//                       mais ruído.
//   ESCURO_VCSEL_PRE / ESCURO_VCSEL_FINAL
//                       pulso do laser. Maior = mais alcance e mais consumo; o chip só aceita
//                       valores pares: pré 12 a 18 e final 8 a 14. Volte para 14 e 10 se o sensor
//                       travar em oponente claro.
//   ESCURO_ALCANCE_MM   até onde o rastreador acredita num alvo preto. Meça afastando um objeto
//                       preto até a leitura piscar e escreva aqui o valor medido. Alto demais = o
//                       robô persegue fantasma; baixo demais = ignora o oponente a meia distância.
//   ESCURO_PD_MAX       chance de ver o preto dentro do alcance. Maior (0,95) = o rastreador confia
//                       mais em cada leitura; menor (0,7) = confia menos e demora mais para decidir.
//   ESCURO_FALSA_FATOR  quantas leituras falsas esperar. Maior = ignora leitura solta e demora a
//                       reagir; menor = reage depressa e se engana mais com reflexo.
//   ESCURO_RUIDO_FATOR  erro de distância esperado. Maior = a caixa do oponente fica mais larga.
//   ESCURO_PERDE_MS     tempo sem ver o oponente até dar como perdido. Maior = continua mirando o
//                       último ponto por mais tempo (bom se o preto pisca); menor = volta a buscar
//                       antes (bom se o oponente sai depressa da frente).
// No perfil CLARO os mesmos ajustes estão em TOF_TIMING_US e ALCANCE_PRETO_MM (acima) e em
// FUSAO_PERDE_MS: mexa neles no mesmo sentido (menos sinal e mais tempo = mais alcance e mais ruído).
// Teste de bancada: comando "perfil claro|escuro" no terminal troca o perfil sem mexer na chave.
// ============================================================================
#define ESCURO_TIMING_US    33000
#define ESCURO_SINAL_MCPS   0.10f
#define ESCURO_VCSEL_PRE    18
#define ESCURO_VCSEL_FINAL  14
#define ESCURO_ALCANCE_MM   700     // alcance em alvo preto no longo alcance (medir como o ALCANCE_PRETO_MM)
#define ESCURO_PD_MAX       0.85f   // chance de ver o preto dentro do alcance (0,95 no perfil claro)
#define ESCURO_FALSA_FATOR  2.5f    // leituras falsas a mais em relação ao perfil claro
#define ESCURO_RUIDO_FATOR  1.5f    // ruído a mais na distância
#define ESCURO_PERDE_MS     180     // tempo sem confirmação até perder o alvo (100 ms no claro)

// ============================================================================
// Localização no dojô (arena.cpp): filtro de partículas do centro do dojô
// ============================================================================
#ifndef ARENA_N
#define ARENA_N              600    // partículas (com 300, o teste_arena achou 10% de "confiável" com erro acima de 10 cm)
#endif
#define ARENA_PASSO_MS       5      // intervalo entre passos do filtro
#define ARENA_D_MIN          120    // distância inicial do centro do robô ao centro do dojô (mm),
#define ARENA_D_MAX          300    // atrás da linha Shikiri e com os sensores de borda no preto
#define ARENA_RUIDO_MOV      0.12f  // erro de escala do modelo das rodas sem encoder (desvio no sorteio de cada partícula)
#define ARENA_SIGMA_OK       70     // incerteza (mm) abaixo da qual a localização guia o robô
#define ARENA_SIGMA_DESCARTE 150    // acima desta incerteza (mm) nenhuma leitura é descartada (trava de segurança;
                                    // quem decide o descarte é a concordância de todas as hipóteses, no arena.h)
#define ARENA_SORTEIO_ESCALA  0.04f  // sorteio na escala de cada partícula copiada na reamostragem
#ifndef ARENA_DISCORDA
#define ARENA_DISCORDA       0      // hipóteses (de 64) que podem discordar de "fora do dojô" no descarte
#endif
#ifndef ARENA_MARGEM_MM
#define ARENA_MARGEM_MM      30     // folga além da borda antes de uma leitura contar como fora
#endif
#define ARENA_CONTATO_MM     80     // leitura do oponente abaixo disso indica contato (movimento incerto)
#define T_TRAS_BRANCO_MS     400    // depois da borda traseira, os ToF de trás ignoram o que passa de 120 mm

// ============================================================================
// Estratégias (Rush, Flank e Fan out), baseadas na proposta do Grupo D e no Blackbook
// ============================================================================
// Comum às três: com o oponente vindo rápido de frente, o robô recebe de frente e empurra com
// o torque todo (o JGB37-520 tem redução alta, e um robô rápido costuma ter pouco torque).
#define APROX_RAPIDA_MMS     300    // oponente chegando acima disso conta como investida
#define APROX_RAPIDA_DEG     45     // e só vale dentro deste ângulo da frente
// Contato: com o oponente centrado e perto, o ToF da frente pode perder a leitura (alvo colado na
// lente); o ataque continua por este tempo sem confirmação, com a borda sempre em primeiro lugar.
#define CONTATO_MM           60
#define T_SEGURA_CONTATO_MS  400
// Ataque de ré: com o oponente colado atrás (centro dele a até DIST_RE_MM do centro do robô e a até
// ANG_RE_DEG da traseira), o robô empurra de ré em vez de girar 180 graus na frente dele.
#define DIST_RE_MM           260
#define ANG_RE_DEG           35
// Volta ao centro: depois da fuga da borda (ou sempre que a localização mostra o robô longe do
// meio), a busca leva o robô de volta para o centro do dojô, girando e procurando no caminho.
#define RETORNO_DIST_MM      150    // abaixo disso o robô já está no meio
#define VEL_RETORNO_MMS      320
// Flank: em vez de ir reto, o robô mira ao lado do oponente e chega pela lateral dele, numa curva
// que fecha conforme a distância cai (espiral). O lado escolhido é o que tem mais espaço até a
// borda; perto o bastante, aberto demais ou depois do tempo máximo, o robô ataca direto.
#define FLANCO_DESVIO_DEG    35
#define FLANCO_FECHA_MM      180    // distância do sensor até a face do oponente para fechar o ataque
#define FLANCO_ABRE_DEG      75     // oponente além deste ângulo: o robô já passou do lado dele
#define T_FLANCO_MAX_MS      1500
#define FLANCO_LIVRE_MIN_MM  220    // espaço mínimo até a borda na direção da curva
// Fan out: a busca oscila o rumo de um lado para o outro e avança devagar, varrendo com os
// sensores da frente uma faixa larga; o ataque é igual ao do Rush.
#define FAN_AMPL_DEG         45
#define FAN_GIRO_DPS         260
#define FAN_VEL_MMS          140
#define FAN_LIVRE_MIN_MM     200    // com menos espaço que isso à frente, o leque não avança
// Aberturas: quando os 5 s acabam com o oponente à vista, cada estratégia chega nele de um jeito.
// O Rush vai direto. O Flank sai num arco largo, mirando ABERTURA_FLANCO_DEG ao lado do oponente,
// mesmo com ele vindo rápido (o arco tira o robô da linha da investida), e entra pela lateral
// dele. Com 60 graus e 1 s, o Flank largando de frente perdeu 30 de 40 lutas simuladas, porque o
// oponente que vem reto pega a lateral do arco; com 75 graus e 0,7 s, perdeu 12. Com 80 graus
// perdeu 5, mas numa luta do teste/sim o arco deixou o robô na borda de frente para uma pessoa,
// e ele passou 4,7 s mirando nela. O Fan out chega em zigue-zague (estado LEQUE): o rumo vai de
// um lado para o outro do oponente, e perto dele o ataque é direto. No dojô de 77 cm os dois
// robôs se veem logo na largada, então sem as aberturas as três estratégias faziam a mesma coisa.
#define ABERTURA_ANG_MAX_DEG  60    // oponente além deste ângulo na largada: a mira vem antes
#define ABERTURA_FLANCO_DEG   75
#define T_ABERTURA_FLANCO_MS  700
#define LEQUE_DESVIO_DEG      30    // o rumo oscila 30 graus para cada lado do oponente
#define T_LEQUE_MEIO_MS       200   // tempo de cada perna do zigue-zague
#define LEQUE_VEL             0.8f  // fração da velocidade máxima no zigue-zague
#define LEQUE_FECHA_MM        140   // perto disso o leque vira ataque direto
#define LEQUE_ANG_MAX_DEG     70
#define T_LEQUE_MAX_MS        1500

// ============================================================================
// Travamento (estados.cpp) e aquecimento dos motores (cinematica.cpp)
// ============================================================================
// Sem encoder e sem medida de corrente, o firmware reconhece dois travamentos:
//   - giro travado: a mira pede giro, a cinemática estima que o robô girou mais de
//     GIRO_TRAVA_DEG em T_GIRO_TRAVA_MS, e o ângulo do oponente quase não mudou. É o oponente
//     encostado na lateral, que segura o giro; o robô se solta (ré curta, ou avanço reto sem
//     espaço atrás) e volta a mirar com a quina livre;
//   - empurrão travado: o ataque em contato passa de T_TRAVA_MS sem a luta acabar. O robô continua
//     empurrando: nas lutas simuladas, soltar ou girar no meio do empurrão entregou o lugar ao
//     oponente e transformou vitórias em derrotas, contra oponentes de força igual ou maior.
#define T_GIRO_TRAVA_MS      600
#define GIRO_TRAVA_DEG       90     // giro estimado na janela, quase todo travado
#define GIRO_TRAVA_GANHO_DEG 15     // melhora mínima do ângulo do oponente para o giro valer
#define T_DESTRAVA_RE_MS     250    // ré para soltar a quina presa
#define T_DESTRAVA_AVANCO_MS 250    // avanço reto para soltar a quina, sem espaço atrás
#define DESTRAVA_LIVRE_MM    140    // espaço mínimo atrás (localização) para a ré
#define T_TRAVA_MS           1600   // empurrão em contato por mais disso aparece como travado
// ----------------------------------------------------------------------------------------------
// O QUE FAZER QUANDO O EMPURRÃO TRAVAR (caixa do rastreador parada no ataque, antitrava.h)
// Descomente UMA linha só. Com todas comentadas, o robô só avisa: o terminal, o painel e o LED
// (PINO_LED_TRAVA, abaixo) mostram "robô travado" e o robô continua empurrando. Nas lutas
// simuladas contra um oponente de 319 rpm nenhuma manobra ganhou mais de uma luta a mais que o
// aviso: a Lateral e o Giro entregaram o lugar ao oponente e as outras três não mudaram nada. Só
// vale a pena ligar uma delas depois de testar no dojô.
//   Só avisa  -> nenhuma linha descomentada (padrão)
//   Lateral   -> ré em arco e volta pela lateral do oponente
//   Torque    -> continua empurrando com um giro suave (desequilibra o oponente)
//   Zigue     -> empurra com giro alternando de lado a cada ANTI_TRAVA_ZIGUE_MS
//   Giro      -> giro no eixo com tensão toda por ANTI_TRAVA_GIRO_MS
//   Pulso     -> solta as rodas por ANTI_TRAVA_SOLTA_MS e empurra de novo, em ciclos
// ANTI_TRAVA 0 (logo abaixo) tira o módulo inteiro, inclusive o aviso. O comando "trava N" do
// terminal troca a manobra sem gravar (0 só avisa, 1 lateral, 2 torque, 3 zigue, 4 giro, 5 pulso).
// ----------------------------------------------------------------------------------------------
//#define ANTI_TRAVA_MANOBRA   1   // Lateral
//#define ANTI_TRAVA_MANOBRA   2   // Torque
//#define ANTI_TRAVA_MANOBRA   3   // Zigue
//#define ANTI_TRAVA_MANOBRA   4   // Giro
//#define ANTI_TRAVA_MANOBRA   5   // Pulso
#ifndef ANTI_TRAVA
#define ANTI_TRAVA           1
#endif
#ifndef ANTI_TRAVA_MANOBRA
#define ANTI_TRAVA_MANOBRA   0     // Só avisa
#endif
#define ANTI_TRAVA_T_MS      1200   // caixa parada no ataque por mais disso: travado
#define ANTI_TRAVA_TOL_MM    25     // tolerância da distância da caixa (erro do rastreador perto de 10 mm)
#define ANTI_TRAVA_TOL_DEG   12     // tolerância do lado da caixa (erro do rastreador de 3 a 8 graus na frente)
#define ANTI_TRAVA_PERTO_MM  60     // menor leitura do oponente até isso (ou contato) conta como encostado
#define ANTI_TRAVA_FORA_MS   150    // saída da tolerância mais curta que isso (salto da estimativa) não zera a janela
#define ANTI_TRAVA_PAUSA_MS  1000   // depois de uma manobra, tempo até a próxima detecção
#define ANTI_TRAVA_LIVRE_MM  150    // espaço mínimo atrás para a ré da manobra lateral
#define ANTI_TRAVA_RE_MS     200    // lateral: ré em arco
#define ANTI_TRAVA_RE_GIRO_DPS 220
#define ANTI_TRAVA_ARCO_MS   400    // lateral: volta em arco pela lateral do oponente
#define ANTI_TRAVA_ARCO_DEG  60
#define ANTI_TRAVA_TORQUE_MS 500    // torque e zigue: empurrão com giro
#define ANTI_TRAVA_TORQUE_DPS 150
#define ANTI_TRAVA_ZIGUE_MS  125    // zigue: tempo de cada lado
#define ANTI_TRAVA_GIRO_MS   200    // giro strike: giro no eixo com tensão toda (sem a rampa de aceleração)
#define ANTI_TRAVA_PULSO_MS  300    // pulso: ciclo de solta e empurra
#define ANTI_TRAVA_SOLTA_MS  20     // rodas soltas em cada pulso
// Modelo térmico de cada motor: corrente = (tensão aplicada - força contraeletromotriz) /
// resistência, com resistência de 12 V / 1,2 A = 10 ohms (tabela do motor) e força
// contraeletromotriz zero com a roda travada. Aquecimento de primeira ordem com capacidade e
// resistência térmicas estimadas para um motor de 37 mm (medir com termômetro no robô montado).
// Acima de MOTOR_QUENTE_K de elevação, as manobras normais ficam limitadas a MOTOR_PWM_QUENTE;
// a fuga da borda nunca é limitada.
#define MOTOR_I_TRAVADO_A    1.2f
#define MOTOR_C_TERM_JK      40.0f  // J/K (estimado; medir)
#define MOTOR_R_TERM_KW      6.0f   // K/W (estimado; medir)
#define MOTOR_QUENTE_K       60.0f  // elevação de temperatura que liga o limite
#define MOTOR_FRIO_K         45.0f  // elevação abaixo da qual o limite sai
#define MOTOR_PWM_QUENTE     0.7f
