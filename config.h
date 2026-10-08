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
// Referencial (o mesmo dos gabaritos em doc/): origem no centro de giro (na bancada, o
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

// 1 escolhe a geometria da bancada (protoboard 165 x 54,7 mm), 0 escolhe o robô (152 x 152 mm)
#define GEOMETRIA_BANCADA 1

// Quantidade de sensores de oponente: 5 (montagem original) ou 7, com um par a mais nos cantos
// de trás (TE e TD, a ±160 graus). A frente continua com os 3 sensores de antes e o FC segue
// sozinho no centro, decidindo o ataque, como pede o capítulo 2 do Sumo Robot Blackbook (número
// ímpar de sensores, com um no centro). Na simulação com o rastreador, o par de trás reduziu os
// ataques que chegam sem ser vistos de 36% para 8% e o tempo médio de busca de 206 para 35 ms,
// sem piorar a precisão na frente (detalhes no README e no guia de 7 sensores).
// O par novo fica no fim da tabela para os 5 sensores antigos manterem o mesmo XSHUT, endereço,
// cor no painel e calibração gravada.
// O valor também pode vir da compilação (-DQTD_TOF=7), como o CI faz.
// Com 7 sensores, o I2C a 100 kHz fica ocupado boa parte do tempo; na placa definitiva, com
// fios curtos, o I2C_CLOCK_HZ pode subir para 400000.
#ifndef QTD_TOF
#define QTD_TOF 5
#endif
#if QTD_TOF != 5 && QTD_TOF != 7
#error "QTD_TOF precisa ser 5 ou 7"
#endif

#if GEOMETRIA_BANCADA
// Protoboard 165 x 54,7 x 10,2 mm com a linha 63 na frente; a origem é o centro dela.
// Medidas da montagem real (01/10): FE e FD a 54,4 mm um do outro, FC a 31,6 mm do FE e a
// 29,9 mm do FD. Com essas três distâncias o FC fica cerca de 14 mm à frente dos cantos e
// 1 mm para a direita. LE e LD ficam a 59,3 mm um do outro, no meio da protoboard.
// TE e TD (7 sensores) ainda não foram montados: a posição abaixo é a prevista no gabarito
// (doc/guia_montagem_sensores_7.pdf), nas quinas de trás da protoboard, com o palito colado por
// fora para não atrapalhar os fios das linhas 1 a 4. Depois de montar, a posição medida
// substitui estes números.
static const CfgToF TOF[] = {
  //  nome   xshut  endereço   x     y    ang   offset
  {  "LE",   13,    0x30,     -1,   30,   90,    0  },   // meio da lateral esquerda
  {  "FE",   14,    0x31,     73,   27,   20,    0  },   // canto frente esquerdo
  {  "FC",   27,    0x32,     87,   -1,    0,    0  },   // centro da frente, decide o ataque
  {  "FD",   26,    0x33,     73,  -27,  -20,    0  },   // canto frente direito
  {  "LD",   25,    0x34,     -2,  -30,  -90,    0  },   // meio da lateral direita
  // XSHUT do TE no GPIO 33 e do TD no GPIO 15. O 15 é pino de boot, mas o nível que ele precisa
  // no boot é o alto, o mesmo do XSHUT com o sensor ligado. O GPIO 12 ficou de fora porque
  // nível alto nele no boot muda a tensão da flash e o ESP32 pode não ligar.
#if QTD_TOF == 7
  {  "TE",   33,    0x35,    -80,   25,  160,    0  },   // quina traseira esquerda (prevista)
  {  "TD",   15,    0x36,    -80,  -25, -160,    0  },   // quina traseira direita (prevista)
#endif
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
// Robô de 152 x 152 mm com a origem no centro. Com 7 sensores, TE e TD ficam nos cantos de
// trás, apontando a ±160 graus, onde os 5 sensores não enxergam nada.
static const CfgToF TOF[] = {
  //  nome   xshut  endereço   x     y    ang   offset
  {  "LE",   13,    0x30,     10,   70,   90,    0  },   // lateral esquerda
  {  "FE",   14,    0x31,     66,   40,   20,    0  },   // frente esquerda
  {  "FC",   27,    0x32,     68,    0,    0,    0  },   // frente centro, decide o ataque
  {  "FD",   26,    0x33,     66,  -40,  -20,    0  },   // frente direita
  {  "LD",   25,    0x34,     10,  -70,  -90,    0  },   // lateral direita
#if QTD_TOF == 7
  {  "TE",   33,    0x35,    -60,   62,  160,    0  },   // canto traseiro esquerdo
  {  "TD",   15,    0x36,    -60,  -62, -160,    0  },   // canto traseiro direito
#endif
};
#define CORPO_FRENTE_MM   76.0f
#define CORPO_TRAS_MM     76.0f
#define CORPO_ESQ_MM      76.0f
#define CORPO_DIR_MM      76.0f
#define CORPO_ALTURA_MM   40.0f
#define LENTE_ALTURA_MM   22.0f
#define GEOMETRIA_NOME    "ROBO"
#endif
#define N_TOF  (sizeof(TOF) / sizeof(TOF[0]))   // quantidade de ToF, no máximo 16

#define TOF_TIMING_US     20000   // 20 ms por medida (33000 aumenta o alcance em alvo preto)
#define DOHYO_DIAMETRO_MM 770     // dohyo RoboCore 1 kg: 77 cm, a faixa branca faz parte dele
// Alcance contado a partir do centro do robô; leitura mais longe vira "nada" (parede,
// mesa, pessoas). Pior caso dentro do dohyo: o robô encostado numa borda (centro a 7,6 cm
// dela) e o oponente encostado na borda oposta, com a face a 77 - 7,6 - 15,2 = 54,2 cm do
// centro do robô. Para o sensor da frente, que fica 8,7 cm à frente do centro, isso dá uns 45 cm.
#define TOF_ALCANCE_MM    (DOHYO_DIAMETRO_MM - OPONENTE_LADO_MM / 2 - OPONENTE_LADO_MM)
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
#define GIRO_MAX_DPS      600     // com motores: giro no eixo com comando 255 (precisa ser medido)
#define ROBO_VEL_MAX_MMS  1500    // com motores: velocidade em linha reta com 255 (precisa ser medida)
#define FUSAO_MASSA_MIN   0.25f   // concentração mínima do mapa (0 a 1) para contar como "visto"
#define FUSAO_EXISTE_MIN  0.5f    // chance mínima de existir oponente para contar como "visto"
                                  // (arena vazia e leitura falsa isolada ficam abaixo disso)
#define MAPA_MS           250     // intervalo de envio do mapa de probabilidade para o painel
#define RASTREADOR_NUCLEO 0       // núcleo do rastreador (o loop fica no 1); -1 roda dentro do loop

// Valores usados apenas quando FUSAO_BAYES é 0
#define FUSAO_CLUSTER_MM  150     // leituras com diferença até esse valor são o mesmo objeto
#define FUSAO_CLUSTER_DEG 45      // e só entre sensores vizinhos, com até 45° entre eles

// ============================================================================
// Sensores de borda: 4 módulos TCRT5000 (com LM393, pinos VCC GND D0 A0)
// O código usa a saída analógica A0 (Blackbook, capítulo 9), porque o D0 do comparador
// não distingue arranhão de linha; o trimpot azul só mexe no D0. Os pinos 34, 35, 36 (VP)
// e 39 (VN) são do ADC1 e servem só como entrada, o que combina com o A0. A alimentação é de
// 3,3 V, assim o A0 nunca passa de 3,3 V. Dohyo: 77 cm, laminado preto e borda branca de
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
  {  "BTE",   36,    -75,   34,  -1,   false },   // trás esquerda, GPIO36 é o VP
  {  "BTD",   39,    -60,  -34,  +1,   false },   // trás direita, GPIO39 é o VN
};
#define BORDA_ALTURA_MM   2.7f     // altura da face do sensor ao chão, só para o painel 3D
#else
static const CfgBorda BORDA[] = {
  {  "BFE",   34,     60,   60,  -1,   true  },
  {  "BFD",   35,     60,  -60,  +1,   true  },
  {  "BTE",   36,    -60,   60,  -1,   false },
  {  "BTD",   39,    -60,  -60,  +1,   false },
};
#define BORDA_ALTURA_MM   4.0f
#endif
#define N_BORDA  (sizeof(BORDA) / sizeof(BORDA[0]))

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
// Motores (driver TB6612FNG). Na bancada o valor de USAR_MOTORES fica em 0.
// ============================================================================
#define USAR_MOTORES      0
#define PINO_PWMA         18
#define PINO_AIN1         19
#define PINO_AIN2         23
#define PINO_PWMB         4
#define PINO_BIN1         16
#define PINO_BIN2         17
// o STBY do TB6612 vai direto no 3V3
#define INVERTE_ESQ       0      // 1 quando o motor esquerdo gira ao contrário
#define INVERTE_DIR       0

// ============================================================================
// Interface. O robô é autônomo: a telemetria sai só pela USB, para o Monitor Serial e para
// o painel (painel_sumo.html), e o firmware não liga WiFi nem Bluetooth.
// ============================================================================
#define PINO_START        32     // botão ligado ao GND (INPUT_PULLUP)
#define PINO_LED          2      // LED azul da placa
#define SERIAL_BAUD       115200
#define STREAM_MS         20     // dados para o painel a 50 Hz, o mesmo ritmo dos sensores

// ============================================================================
// Estratégia: velocidades de -255 a 255 e tempos em ms
// ============================================================================
#define T_CONTAGEM_MS     5000   // regra RoboCore: 5 s depois do comando de início
#define VEL_ATAQUE        255
#define VEL_MIRA_IN       80     // velocidade da roda de dentro quando a mira é corrigida em curva
#define VEL_GIRO          200
#define VEL_BUSCA         120
#define VEL_RECUO         220
#define T_RECUO_EXTRA_MS  10     // folga depois que a borda some (Blackbook, capítulo 10)
#define T_GIRO_BORDA_MS   250    // tempo de giro na borda, cerca de 135 graus a VEL_GIRO (precisa ser medido)
#define T_BUSCA_GIRO_MS   1500   // BUSCA: tempo girando nos pontos cegos sem achar nada
#define T_BUSCA_AVANCO_MS 250    // BUSCA: tempo do avanço curto que muda o ponto de vista
#define MODO_DEFENSIVO    0      // 1 faz o robô só mirar e nunca avançar
