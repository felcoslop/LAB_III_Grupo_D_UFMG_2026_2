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
// O par novo fica no fim da tabela para os 5 sensores antigos manterem o mesmo endereço, cor no
// painel e calibração gravada.
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
  {  "LE",   17,    0x30,     -1,   30,   90,    0  },   // meio da lateral esquerda
  {  "FE",   23,    0x31,     73,   27,   20,    0  },   // canto frente esquerdo
  {  "FC",   19,    0x32,     87,   -1,    0,    0  },   // centro da frente, decide o ataque
  {  "FD",   18,    0x33,     73,  -27,  -20,    0  },   // canto frente direito
  {  "LD",    5,    0x34,     -2,  -30,  -90,    0  },   // meio da lateral direita
  // XSHUT conforme a pinagem da placa nova (Lab3-Liga). O GPIO 5 é pino de boot e precisa estar
  // em nível alto no boot, que é o nível do XSHUT com o sensor ligado (pull-up do módulo).
#if QTD_TOF == 7
  {  "TE",   16,    0x35,    -80,   25,  160,    0  },   // quina traseira esquerda (prevista)
  {  "TD",    4,    0x36,    -80,  -25, -160,    0  },   // quina traseira direita (prevista)
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
  {  "LE",   17,    0x30,     10,   70,   90,    0  },   // lateral esquerda
  {  "FE",   23,    0x31,     66,   40,   20,    0  },   // frente esquerda
  {  "FC",   19,    0x32,     68,    0,    0,    0  },   // frente centro, decide o ataque
  {  "FD",   18,    0x33,     66,  -40,  -20,    0  },   // frente direita
  {  "LD",    5,    0x34,     10,  -70,  -90,    0  },   // lateral direita
#if QTD_TOF == 7
  {  "TE",   16,    0x35,    -60,   62,  160,    0  },   // canto traseiro esquerdo
  {  "TD",    4,    0x36,    -60,  -62, -160,    0  },   // canto traseiro direito
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
                                  // com motores, o giro e o avanço vêm da cinemática (cinematica.cpp)
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
static const CfgBorda BORDA[] = {
  {  "BFE",   34,     60,   60,  -1,   true  },
  {  "BFD",   35,     60,  -60,  +1,   true  },
  {  "BTE",   32,    -60,   60,  -1,   false },
  {  "BTD",   33,    -60,  -60,  +1,   false },
};
#define BORDA_ALTURA_MM   4.0f
#define BORDA_ATE_RODA_MM 60.0f    // menor distância em x entre um sensor de borda e o eixo das rodas
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
#define BITOLA_MM         132.0f   // distância entre os centros das duas rodas (medir no chassi)
#define ACEL_MAX_MMS2     6000.0f  // limite das manobras normais, abaixo da tração (atrito ~1)
#define FREIO_MIN_MMS2    4400.0f  // pior frenagem: roda patinando, atrito 0,7 e 80% do peso nas rodas
#define BORDA_FAIXA_MM    25.0f    // largura da faixa branca da borda (regra RoboCore)

// ============================================================================
// Interface. O robô é autônomo: a telemetria sai só pela USB, para o Monitor Serial e para
// o painel (painel_sumo.html), e o firmware não liga WiFi nem Bluetooth.
// ============================================================================
// Botão de start no GPIO 0, o mesmo do botão BOOT da placa, que já tem pull-up. O S1 do
// esquema fica entre o GPIO 0 e o GND, em paralelo com o BOOT. Com o botão apertado na hora de
// ligar, o ESP32 entra no modo de gravação, então o botão só pode ser usado com o robô ligado.
#define PINO_START        0      // nível baixo indica botão apertado (INPUT_PULLUP)
// A chave DIP de 4 vias usa os GPIOs 12, 14, 2 e 15 e liga cada pino no 3V3. Por isso o GPIO 2
// não serve mais para o LED: com a chave ligada e o pino em LOW, o 3V3 ficaria em curto com o
// pino. O valor -1 desliga o LED de status. Cuidados no boot: DIP 1 (GPIO 12) desligada na hora
// de ligar o ESP32, porque nível alto no GPIO 12 muda a tensão da flash e a placa não liga, e
// DIP 3 (GPIO 2) desligada na hora de gravar o firmware.
#define PINO_LED          -1
#define SERIAL_BAUD       115200
#define STREAM_MS         20     // dados para o painel a 50 Hz, o mesmo ritmo dos sensores

// ============================================================================
// Estratégia: avanço em mm/s, giro em graus/s (positivo é para a esquerda) e tempos em ms.
// A cinemática transforma cada par (avanço, giro) na velocidade e no PWM de cada roda. O
// ataque e a fuga da borda usam a velocidade máxima (VEL_MAX_MMS, calculada no cinematica.h).
// ============================================================================
#define T_CONTAGEM_MS     5000   // regra RoboCore: 5 s depois do comando de início
#define VEL_BUSCA_MMS     240    // avanço curto da BUSCA, cerca de metade da velocidade máxima
#define GIRO_BUSCA_DPS    200    // giro da BUSCA, cerca de metade do giro máximo
#define K_MIRA            6.0f   // mira: graus/s de giro para cada grau de erro (erro cai a 37% em ~0,17 s)
#define ANG_MIRA_CURVA    40     // até esse ângulo a mira é uma curva; acima, giro no eixo
#define GIRO_MIRA_MAX_DPS 370    // giro máximo na mira, 90% do giro máximo do robô
#define GIRO_BORDA_DPS    370    // giro de recuperação depois da borda
#define ANG_GIRO_BORDA    135    // tamanho desse giro em graus, estimado pela cinemática
#define T_GIRO_BORDA_MAX_MS 900  // segurança: o giro termina mesmo se a estimativa não chegar lá
#define T_RECUO_EXTRA_MS  10     // folga depois que a borda some (Blackbook, capítulo 10)
#define T_BUSCA_GIRO_MS   1500   // BUSCA: tempo girando nos pontos cegos sem achar nada
#define T_BUSCA_AVANCO_MS 250    // BUSCA: tempo do avanço curto que muda o ponto de vista
#define MODO_DEFENSIVO    0      // 1 faz o robô só mirar e nunca avançar
