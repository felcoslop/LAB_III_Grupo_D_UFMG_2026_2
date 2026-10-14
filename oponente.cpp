// oponente.cpp
// Módulo dos sensores de oponente VL53L0X (placas CJVL53L0XV2 / GY-530), 5 ou 7 conforme o QTD_TOF.
// Ele cuida de quatro coisas: ligar os sensores e dar um endereço I2C para cada um, ler as
// distâncias, descartar o que cai fora do dojô e juntar as leituras em um único oponente.
//
// Divisão entre os núcleos (SENSORES_NUCLEO): a leitura pelo I2C, o descarte e o passo do
// rastreador rodam numa tarefa do FreeRTOS no núcleo 0 (tof_ciclo); o loop, no núcleo 1, só lê a
// estimativa pronta (oponente_funde). O I2C passa por um mutex, porque alguns comandos do
// terminal (init, scan) também usam o barramento, e as mensagens da tarefa saem pelo loop, já
// que o Log não pode ser escrito pelos dois núcleos ao mesmo tempo. Sem a tarefa (SENSORES_NUCLEO
// -1, ou no simulador do computador), o tof_ciclo roda dentro do loop.
//
// Cuidados com as placas clone, medidos na bancada: os pinos SDA e SCL têm pull-up para
// 2,8 V e não têm conversor de nível, então funcionam direto no ESP32 (3,3 V) mas não no
// Arduino de 5 V. O XSHUT também não tem conversor, por isso o código nunca coloca nível
// alto nele: para desligar o pino vira saída em LOW e para ligar vira entrada com o pull-up
// interno do ESP32 (cerca de 45 kOhm para 3,3 V), que fica abaixo do máximo do chip (3,6 V).

#include "oponente.h"
#include "config.h"
#include "telemetria.h"
#include "rastreador.h"
#include "cinematica.h"
#include "arena.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <Preferences.h>
#include <math.h>

#if defined(ARDUINO_ARCH_ESP32) && SENSORES_NUCLEO >= 0
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#define COM_TAREFA 1
static TaskHandle_t      tarefa = nullptr;
static SemaphoreHandle_t mutexI2C = nullptr;
static portMUX_TYPE      travaEv = portMUX_INITIALIZER_UNLOCKED;
#define TRAVA_EV()    portENTER_CRITICAL(&travaEv)
#define DESTRAVA_EV() portEXIT_CRITICAL(&travaEv)
#else
#define COM_TAREFA 0
#define TRAVA_EV()
#define DESTRAVA_EV()
#endif

static VL53L0X  sensor[N_TOF];        // um objeto da biblioteca Pololu para cada sensor
static volatile bool ok[N_TOF];       // sensor respondendo
static volatile int16_t bruto[N_TOF], filtrado[N_TOF];   // leitura crua e leitura corrigida com mediana
static int16_t  hist[N_TOF][3];       // últimas 3 leituras de cada sensor, para a mediana
static uint8_t  hIdx[N_TOF];          // posição atual no histórico circular
static uint32_t tUlt[N_TOF];          // momento da última medida nova
static volatile uint32_t tColado[N_TOF];   // última medida abaixo de TOF_MIN_MM: algo encostado no sensor
static bool     obstruido[N_TOF];      // leitura colada já parado antes da luta: fio, lâmina ou poeira na lente
static volatile uint16_t seq[N_TOF];  // contador de medidas novas
static uint8_t  errI2C[N_TOF];        // erros de I2C seguidos (sensor que sumiu do barramento)
static int8_t   idxCentro = -1;       // índice do sensor central

// calibração: leitura corrigida = ganho * bruto + offset
static float    calGanho[N_TOF];
static float    calOffset[N_TOF];
static bool     calNvs[N_TOF];        // true quando a calibração veio do comando "cal" e está gravada
static Preferences prefs;             // acesso à memória NVS do ESP32

// estado da fusão (núcleo 1)
static float    ultAng = 0;
static int8_t   ultLado = 0;
static uint32_t tVisto = 0;
static bool     jaViu = false;
static uint32_t passoAnt = 0;         // último passo do rastreador já usado na aproximação
static float    rhoAnt = 0, aprox = 0;
static uint32_t tRhoAnt = 0;

// dados para o rastreador: última medida corrigida de cada sensor, sem a mediana
// (-1 significa que mediu e não tem nada no alcance; -2 significa sem medida válida)
static volatile int16_t ultZ[N_TOF];
static int16_t  alcance[N_TOF];      // maior leitura que ainda fica dentro do raio TOF_ALCANCE_MM em volta do robô
static uint16_t seqUsado[N_TOF];     // última medida já entregue ao rastreador
static bool     usaBayes = false;    // true quando o rastreador conseguiu memória e está ativo
static uint32_t tPasso = 0;          // momento do último passo do rastreador

// perfil do oponente (claro ou escuro): orçamento de tempo de cada medida e tempo para perder o alvo
static volatile bool escuroAtual = false, escuroPedido = false;
static volatile uint32_t periodoMs = TOF_TIMING_US / 1000;
static volatile uint32_t perdeMs = FUSAO_PERDE_MS;

// pedidos do núcleo 1 para a tarefa e avisos da tarefa para o núcleo 1
static volatile bool     manutencaoLiberada = false;
static uint32_t          tRetry = 0;
static volatile uint32_t tTrasBranco = 0;          // última vez em que a borda de trás estava no branco
static volatile bool     trasBrancoJa = false;
static uint16_t          evCaiu = 0, evVoltou = 0;  // sensores que caíram ou voltaram (para o Log)
static volatile uint16_t descartes = 0;             // leituras descartadas por cair fora do dojô

static const char* MOTIVO = "FALHOU (VCC/GND/SDA/SCL frouxo ou fio XSHUT deste sensor)";

// XSHUT: saída LOW desliga o sensor; entrada com pull-up liga sem nunca forçar nível alto
static void xshutDesliga(uint8_t i) { pinMode(TOF[i].xshut, OUTPUT); digitalWrite(TOF[i].xshut, LOW); }
static void xshutLiga(uint8_t i)    { pinMode(TOF[i].xshut, INPUT_PULLUP); }

// limpeza das leituras de um sensor (usada no boot e quando ele cai ou volta)
static void zeraLeitura(uint8_t i) {
  errI2C[i] = 0;
  bruto[i] = filtrado[i] = -1;
  ultZ[i] = -2;
  hist[i][0] = hist[i][1] = hist[i][2] = -1;
  tUlt[i] = millis();
}

// Perfil do chip. No claro só o orçamento muda (os outros valores ficam os de fábrica, como
// sempre foram); no escuro entra o longo alcance. O chip precisa estar parado para receber os
// pulsos novos do laser, porque a troca refaz a calibração de referência.
static void aplicaPerfilChip(uint8_t i, bool escuro) {
  if (escuro) {
    sensor[i].setSignalRateLimit(ESCURO_SINAL_MCPS);
    sensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, ESCURO_VCSEL_PRE);
    sensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, ESCURO_VCSEL_FINAL);
    sensor[i].setMeasurementTimingBudget(ESCURO_TIMING_US);
  } else {
    sensor[i].setSignalRateLimit(0.25f);
    sensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    sensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
    sensor[i].setMeasurementTimingBudget(TOF_TIMING_US);
  }
}

// Esta função liga um sensor, que acorda no endereço de fábrica 0x29, e passa ele para o
// endereço da tabela. Ela só funciona se nenhum outro sensor estiver no 0x29 naquele momento
// (os que falharam continuam com XSHUT em LOW).
static bool iniciaUm(uint8_t i) {
  xshutDesliga(i);
  delay(5);
  xshutLiga(i);
  delay(15);                          // o boot do sensor leva menos de 2 ms, 15 ms dá folga
  sensor[i] = VL53L0X();              // o objeto volta a apontar para o 0x29
  sensor[i].setTimeout(100);
  if (!sensor[i].init()) { xshutDesliga(i); return false; }
  sensor[i].setAddress(TOF[i].endereco);
  if (escuroAtual) aplicaPerfilChip(i, true);
  else sensor[i].setMeasurementTimingBudget(TOF_TIMING_US);   // perfil claro: exatamente como antes
  sensor[i].startContinuous(0);       // modo contínuo: o sensor mede sozinho, o código só busca o resultado
  return true;
}

// ==================== calibração (NVS é a memória que não apaga ao desligar) ====================

// O laço percorre os sensores e procura as chaves "<NOME>_g" e "<NOME>_o" na NVS;
// sem calibração gravada, o sensor usa ganho 1 e o offset do config.h.
static void calCarrega() {
  prefs.begin("tofcal", true);
  for (uint8_t i = 0; i < N_TOF; i++) {
    char kg[12], ko[12];
    snprintf(kg, sizeof(kg), "%s_g", TOF[i].nome);
    snprintf(ko, sizeof(ko), "%s_o", TOF[i].nome);
    if (prefs.isKey(kg) && prefs.isKey(ko)) {
      calGanho[i] = prefs.getFloat(kg, 1.0f);
      calOffset[i] = prefs.getFloat(ko, 0.0f);
      calNvs[i] = true;
    } else {
      calGanho[i] = 1.0f;
      calOffset[i] = TOF[i].offset_mm;
      calNvs[i] = false;
    }
  }
  prefs.end();
}

// gravação da calibração calculada no debug.cpp (comando "cal"), que já passa a valer
void tof_cal_define(uint8_t i, float ganho, float offset) {
  if (i >= N_TOF) return;
  char kg[12], ko[12];
  snprintf(kg, sizeof(kg), "%s_g", TOF[i].nome);
  snprintf(ko, sizeof(ko), "%s_o", TOF[i].nome);
  prefs.begin("tofcal", false);
  prefs.putFloat(kg, ganho);
  prefs.putFloat(ko, offset);
  prefs.end();
  calGanho[i] = ganho; calOffset[i] = offset; calNvs[i] = true;
  hist[i][0] = hist[i][1] = hist[i][2] = -1;   // o histórico antigo não vale mais
}

// remoção da calibração de um sensor (comando "calzera")
void tof_cal_zera(uint8_t i) {
  if (i >= N_TOF) return;
  char kg[12], ko[12];
  snprintf(kg, sizeof(kg), "%s_g", TOF[i].nome);
  snprintf(ko, sizeof(ko), "%s_o", TOF[i].nome);
  prefs.begin("tofcal", false);
  prefs.remove(kg);
  prefs.remove(ko);
  prefs.end();
  calGanho[i] = 1.0f; calOffset[i] = TOF[i].offset_mm; calNvs[i] = false;
  hist[i][0] = hist[i][1] = hist[i][2] = -1;
}

float tof_cal_ganho(uint8_t i)  { return i < N_TOF ? calGanho[i] : 1.0f; }
float tof_cal_offset(uint8_t i) { return i < N_TOF ? calOffset[i] : 0.0f; }
bool  tof_cal_gravada(uint8_t i){ return i < N_TOF && calNvs[i]; }

// ==================== I2C compartilhado com o terminal ====================
void tof_i2c_pega() {
#if COM_TAREFA
  if (mutexI2C) xSemaphoreTake(mutexI2C, portMAX_DELAY);
#endif
}
void tof_i2c_solta() {
#if COM_TAREFA
  if (mutexI2C) xSemaphoreGive(mutexI2C);
#endif
}

static void avisaCaiu(uint8_t i)   { TRAVA_EV(); evCaiu |= (1u << i); evVoltou &= ~(1u << i); DESTRAVA_EV(); }
static void avisaVoltou(uint8_t i) { TRAVA_EV(); evVoltou |= (1u << i); evCaiu &= ~(1u << i); DESTRAVA_EV(); }

// ==================== tarefa dos sensores ====================
static void tofCiclo();
#if COM_TAREFA
static void tarefaSensores(void*) {
  for (;;) {
    xSemaphoreTake(mutexI2C, portMAX_DELAY);
    tofCiclo();
    xSemaphoreGive(mutexI2C);
    vTaskDelay(1);                     // 1 ms: deixa o núcleo 0 respirar (cão de guarda do FreeRTOS)
  }
}
#endif

// Inicialização dos ToF, chamada no percepcao_init (núcleo 1, antes de a tarefa existir).
uint8_t tof_init() {
  calCarrega();
  for (uint8_t i = 0; i < N_TOF; i++) alcance[i] = (int16_t)rastreador_alcance_sensor(i);
  // etapa 1: todos desligados, assim só um sensor responde no 0x29 de cada vez
  for (uint8_t i = 0; i < N_TOF; i++) { xshutDesliga(i); ok[i] = false; zeraLeitura(i); }
  delay(20);

  // etapa 2: o laço liga um sensor por vez, inicia no 0x29 e troca o endereço
  uint8_t nOk = 0;
  for (uint8_t i = 0; i < N_TOF; i++) {
    ok[i] = iniciaUm(i);
    zeraLeitura(i);
    if (ok[i]) nOk++;
    Log.printf("# %-3s XSHUT=GPIO%-2d end=0x%02X  %s\n", TOF[i].nome, TOF[i].xshut, TOF[i].endereco,
                  ok[i] ? "OK" : MOTIVO);
  }

  // o sensor central é o de ângulo mais perto de 0 (Blackbook, capítulo 2: ele decide o ataque)
  int16_t menor = 999;
  for (uint8_t i = 0; i < N_TOF; i++)
    if (abs(TOF[i].ang) < menor) { menor = abs(TOF[i].ang); idxCentro = i; }

#if FUSAO_BAYES
  // o rastreador é criado só uma vez; num segundo tof_init o mapa apenas recomeça
  if (!usaBayes) {
    uint32_t t0 = millis();
    usaBayes = rastreador_init();
    if (usaBayes) Log.printf("# rastreador bayesiano: oponente %d mm, mapa %dx%d pronto em %lu ms\n",
                             OPONENTE_LADO_MM, RAST_NB, RAST_NR, (unsigned long)(millis() - t0));
    else          Log.println(F("! rastreador: sem memoria, usando a fusao simples"));
  } else rastreador_zera();
#endif
#if COM_TAREFA
  if (!tarefa) {
    mutexI2C = xSemaphoreCreateMutex();
    // prioridade 2, acima da tarefa ociosa; o núcleo 0 fica livre porque o firmware não liga o rádio.
    // Pilha de 8 KB: as cópias da localização (Arena, 0,5 KB cada) e o mapa do rastreador passam
    // por ela, além do printf do Log.
    if (!mutexI2C || xTaskCreatePinnedToCore(tarefaSensores, "sensores", 8192, nullptr, 2, &tarefa, SENSORES_NUCLEO) != pdPASS) {
      tarefa = nullptr;
      Log.println(F("! tarefa dos sensores nao criada: leitura dentro do loop"));
    } else Log.printf("# ToF e rastreador no nucleo %d, loop no nucleo %d\n", SENSORES_NUCLEO, xPortGetCoreID());
  }
#endif
  return nOk;
}

// nova tentativa nos sensores que falharam (com o I2C já reservado)
static uint8_t reiniciaFalhos() {
  uint8_t recuperados = 0;
  for (uint8_t i = 0; i < N_TOF; i++) {
    if (ok[i]) continue;
    if (iniciaUm(i)) { zeraLeitura(i); ok[i] = true; recuperados++; avisaVoltou(i); }
  }
  return recuperados;
}

// comando "init" do terminal: tentativa imediata, com o I2C reservado
uint8_t tof_reinicia_falhos() {
  tof_i2c_pega();
  uint8_t n = reiniciaFalhos();
  tof_i2c_solta();
  return n;
}

// o loop libera a nova tentativa automática só com o robô parado (religar bloqueia uns 20 ms por sensor)
void tof_libera_manutencao(bool sim) { manutencaoLiberada = sim; }

// pedido de perfil (núcleo 1); a tarefa aplica no próximo ciclo
void tof_perfil(bool escuro) { escuroPedido = escuro; }
bool tof_perfil_escuro() { return escuroAtual; }

bool oponente_borda_tras_recente() { return trasBrancoJa && millis() - tTrasBranco < T_TRAS_BRANCO_MS; }
void oponente_borda_tras(bool branco) {
  if (branco) { tTrasBranco = millis(); trasBrancoJa = true; }
}

// mediana de 3 valores com três trocas, sem ordenar vetor
static int16_t mediana3(int16_t a, int16_t b, int16_t c) {
  if (a > b) { int16_t t = a; a = b; b = t; }
  if (b > c) { int16_t t = b; b = c; c = t; }
  if (a > b) { int16_t t = a; a = b; b = t; }
  return b;
}

// Leitura dos sensores que já têm medida nova (uma passada por todos).
static void leSensores() {
  uint32_t agora = millis();
  for (uint8_t i = 0; i < N_TOF; i++) {
    if (!ok[i]) continue;
    uint32_t desde = agora - tUlt[i];

    // leitura escalonada: o sensor só é consultado perto da hora de ter medida nova,
    // o que economiza o barramento I2C
    if (desde + 3 < periodoMs) continue;

    uint8_t st = sensor[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if (sensor[i].last_status != 0) {               // o sensor não respondeu no I2C
      if (errI2C[i] < 255) errI2C[i]++;
      if (desde > TOF_TIMEOUT_MS) { bruto[i] = -1; filtrado[i] = -1; ultZ[i] = -2; }
      // O sensor só vira FALHOU com erro repetido de verdade. Uma pausa longa (por exemplo,
      // religando outro sensor com o comando init) não derruba um sensor bom.
      if (errI2C[i] >= 10 && desde > TOF_PERDIDO_MS) {
        ok[i] = false;
        xshutDesliga(i);                            // o sensor volta desligado para o 0x29 e pode ser religado depois
        zeraLeitura(i);
        avisaCaiu(i);
      }
      continue;
    }
    errI2C[i] = 0;
    if ((st & 0x07) == 0) {                         // o sensor respondeu, mas a medida nova ainda não saiu
      if (desde > TOF_TIMEOUT_MS) { bruto[i] = -1; filtrado[i] = -1; ultZ[i] = -2; }
      continue;
    }
    // a distância fica no registrador RESULT_RANGE_STATUS + 10; depois a interrupção é limpa
    uint16_t r = sensor[i].readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    sensor[i].writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    tUlt[i] = agora;

    int16_t c = -1;
    if (r > 0 && r < 8000) {                                // 8190 e 8191 significam nada à frente
      bruto[i] = r;
      c = (int16_t)lroundf(calGanho[i] * r + calOffset[i]); // aplicação da calibração
      ultZ[i] = c > alcance[i] ? -1 : c;
      if (c < TOF_MIN_MM) {
        // Leitura abaixo do mínimo do VL53L0X: algo encostado no sensor. Com o robô parado antes
        // da luta não existe oponente a menos de 10 cm (linha Shikiri), então ali é obstrução e
        // o sensor perde só esse tipo de leitura até ler normal de novo. Na luta, é o oponente
        // colado naquele lado: a leitura entra no rastreador como TOF_MIN_MM, e o robô sabe onde
        // ele está mesmo sem distância medida.
        if (manutencaoLiberada) obstruido[i] = true;
        if (obstruido[i]) ultZ[i] = -2;
        else { ultZ[i] = TOF_MIN_MM; tColado[i] = agora | 1; c = TOF_MIN_MM; }
      } else obstruido[i] = false;
    }
    else { bruto[i] = -1; ultZ[i] = r == 0 ? -2 : -1; if (r != 0) obstruido[i] = false; }

    // histórico circular de 3 leituras: a mediana some com uma leitura falsa isolada
    hist[i][hIdx[i]] = c;
    hIdx[i] = (hIdx[i] + 1) % 3;
    int16_t m = mediana3(hist[i][0], hist[i][1], hist[i][2]);
    filtrado[i] = (m >= TOF_MIN_MM && m <= alcance[i]) ? m : -1;
    __sync_synchronize();                           // leitura pronta antes do contador mudar
    seq[i]++;
  }
}

// Uma leitura vale como oponente só se cair dentro do dojô. Quando todas as hipóteses da
// localização (arena.h) colocam o ponto além da borda, a leitura vira "sem informação" (nem
// confirma nem desmente nada). Sem essa concordância, vale a regra simples: com a borda de trás no branco há pouco tempo, os ToF de
// trás não podem ver oponente a mais de 120 mm (atrás da faixa só existe o lado de fora).
static bool foraDoDojo(uint8_t i, int16_t z, const Arena& a, uint32_t agora) {
  float ang = TOF[i].ang * (float)DEG_TO_RAD;
  float qx = TOF[i].x + z * cosf(ang), qy = TOF[i].y + z * sinf(ang);
  if (arena_fora(a, qx, qy)) return true;
  if (trasBrancoJa && agora - tTrasBranco < T_TRAS_BRANCO_MS && abs(TOF[i].ang) >= 120 && z > 120) return true;
  return false;
}

// Passo do rastreador a cada medida nova (PERIODO): cada sensor entra com "está a z mm" ou
// "no meu cone não tem nada", menos as leituras de fora do dojô.
static void passoRastreador() {
  uint32_t agora = millis();
  if (agora - tPasso < periodoMs) return;
  float dt = (agora - tPasso) * 0.001f;
  tPasso = agora;
  Arena a;
  arena_le(a);
  int16_t z[N_TOF];
  bool novo[N_TOF];
  for (uint8_t i = 0; i < N_TOF; i++) {
    novo[i] = ok[i] && seq[i] != seqUsado[i] && ultZ[i] != -2;
    seqUsado[i] = seq[i];
    z[i] = (ok[i] && ultZ[i] > 0) ? ultZ[i] : -1;
    if (z[i] > 0 && foraDoDojo(i, z[i], a, agora)) { novo[i] = false; z[i] = -1; descartes++; }
  }
#if USAR_MOTORES
  // com motores, o giro e o avanço vêm do modelo do acionamento (cinematica.cpp), que já
  // conta a rampa e o atraso das rodas, e o mapa gira junto
  rastreador_passo(z, novo, cinematica_w_dps(), true, cinematica_v_mms(), dt, &a);
#else
  // na bancada o robô é girado na mão, sem aviso, e esse giro entra como incerteza
  rastreador_passo(z, novo, 0, false, 0, dt, &a);
#endif
}

// Um ciclo completo da tarefa dos sensores (núcleo 0), com o I2C reservado.
static void tofCiclo() {
  if (escuroPedido != escuroAtual) {              // troca de perfil pedida pela chave DIP 4
    bool e = escuroPedido;
    for (uint8_t i = 0; i < N_TOF; i++) {
      if (!ok[i]) continue;
      sensor[i].stopContinuous();
      aplicaPerfilChip(i, e);
      sensor[i].startContinuous(0);
      zeraLeitura(i);
    }
    escuroAtual = e;
    periodoMs = (e ? ESCURO_TIMING_US : TOF_TIMING_US) / 1000;
    perdeMs = e ? ESCURO_PERDE_MS : FUSAO_PERDE_MS;
    if (usaBayes) { rastreador_perfil(e); rastreador_zera(); }
  }
  leSensores();
  if (usaBayes) passoRastreador();
  if (manutencaoLiberada && millis() - tRetry >= TOF_RETRY_MS) {
    tRetry = millis();
    bool falta = false;
    for (uint8_t i = 0; i < N_TOF; i++) if (!ok[i]) falta = true;
    if (falta) reiniciaFalhos();
  }
}

// Chamada em toda volta do loop. Com a tarefa no núcleo 0 não faz nada; sem ela, o ciclo roda aqui.
void tof_atualiza() {
#if COM_TAREFA
  if (tarefa) return;
#endif
  tofCiclo();
}

bool     tof_ok(uint8_t i)    { return i < N_TOF && ok[i]; }
int16_t  tof_mm(uint8_t i)    { return i < N_TOF ? filtrado[i] : -1; }
int16_t  tof_bruto(uint8_t i) { return i < N_TOF ? bruto[i] : -1; }
int16_t  tof_cru(uint8_t i)   { return i < N_TOF ? ultZ[i] : -2; }
uint16_t tof_seq(uint8_t i)   { __sync_synchronize(); return i < N_TOF ? seq[i] : 0; }
int8_t   tof_centro()         { return idxCentro; }
uint16_t tof_descartes()      { return descartes; }
uint16_t tof_obstruidos() {
  uint16_t m = 0;
  for (uint8_t i = 0; i < N_TOF; i++) if (obstruido[i]) m |= (1u << i);
  return m;
}
uint16_t tof_colados(uint16_t janela_ms) {
  uint16_t m = 0;
  uint32_t agora = millis();
  for (uint8_t i = 0; i < N_TOF; i++) {
    uint32_t t = tColado[i];
    if (t && agora - t <= janela_ms) m |= (1u << i);
  }
  return m;
}

// máscara de bits com os sensores que estão OK (usada na saúde da percepção)
uint16_t tof_mascara_ok() {
  uint16_t m = 0;
  for (uint8_t i = 0; i < N_TOF; i++) if (ok[i]) m |= (1u << i);
  return m;
}

// busca de um sensor pelo nome, sem diferenciar maiúscula de minúscula (usada nos comandos)
int8_t tof_indice(const char* nome) {
  for (uint8_t i = 0; i < N_TOF; i++) if (strcasecmp(nome, TOF[i].nome) == 0) return i;
  return -1;
}

// Mensagens da tarefa, impressas pelo loop (núcleo 1).
static void imprimeAvisos() {
  TRAVA_EV();
  uint16_t c = evCaiu, v = evVoltou;
  evCaiu = evVoltou = 0;
  DESTRAVA_EV();
  for (uint8_t i = 0; i < N_TOF; i++) {
    if (c & (1u << i)) Log.printf("! %s parou de responder\n", TOF[i].nome);
    if (v & (1u << i)) Log.printf("# %s recuperado (0x%02X)\n", TOF[i].nome, TOF[i].endereco);
  }
}

// ==================== fusão simples (FUSAO_BAYES 0) ====================
// Cada leitura vira um ponto (x,y) no referencial do robô. Leituras do mesmo objeto
// (distância parecida e sensores vizinhos) viram um ponto médio, com peso maior para quem
// está mais perto, e esse ponto dá um ângulo. Como a origem é o centro de giro, esse ângulo
// é exatamente o quanto o robô precisa girar.
static void fundeSimples(Oponente& op) {
  uint32_t agora = millis();
  float px[N_TOF], py[N_TOF];
  int16_t f[N_TOF];
  int8_t perto = -1;
  int16_t dPerto = 32767;

  // primeiro laço: converte as leituras em pontos e acha o sensor que vê mais perto
  for (uint8_t i = 0; i < N_TOF; i++) {
    int16_t d = f[i] = filtrado[i];
    if (d <= 0) continue;
    float a = TOF[i].ang * DEG_TO_RAD;
    px[i] = TOF[i].x + d * cosf(a);
    py[i] = TOF[i].y + d * sinf(a);
    if (d < dPerto) { dPerto = d; perto = i; }
  }

  if (perto >= 0) {
    float sx = 0, sy = 0, sw = 0;
    uint16_t mask = 0;
    // segundo laço: média ponderada só dos sensores que veem o mesmo objeto
    for (uint8_t i = 0; i < N_TOF; i++) {
      if (f[i] <= 0) continue;
      // mesmo objeto: distância parecida e sensores vizinhos (frente e lateral ficam separadas)
      bool distParecida = abs(f[i] - dPerto) <= FUSAO_CLUSTER_MM;
      bool vizinho      = abs(TOF[i].ang - TOF[perto].ang) <= FUSAO_CLUSTER_DEG;
      if (distParecida && vizinho) {
        float w = 1.0f / (f[i] + 30.0f);               // quem está perto pesa mais
        sx += w * px[i]; sy += w * py[i]; sw += w;
        mask |= (1u << i);
      }
    }
    op.visto   = true;
    op.angulo  = atan2f(sy / sw, sx / sw) * RAD_TO_DEG;
    op.dist_mm = dPerto;
    op.mascara = mask;
    // centro do oponente aproximado: ponto visto mais meio lado na mesma direção
    float rr = sqrtf(sx * sx + sy * sy) / sw + OPONENTE_LADO_MM / 2;
    op.x = rr * cosf(op.angulo * DEG_TO_RAD);
    op.y = rr * sinf(op.angulo * DEG_TO_RAD);
    op.sigma = 0;
    op.confianca = 1;
    op.existe = 1;
    op.busca_ganho = 0;
    // Centrado exige o sensor central vendo. Com o central com defeito, o critério cai para
    // "ângulo pequeno" e o robô continua conseguindo atacar.
    bool centroOk = idxCentro >= 0 && ok[idxCentro];
    bool centroVe = centroOk ? (mask & (1u << idxCentro)) != 0 : true;
    op.centrado = centroVe && fabsf(op.angulo) <= CENTRO_DEG;
    ultAng = op.angulo;
    ultLado = op.angulo >= 0 ? 1 : -1;
    tVisto = agora;
    jaViu = true;
  } else if (jaViu && agora - tVisto <= perdeMs) {
    op.visto = true;                                   // o alvo fica segurado um instante se o sensor piscar
    op.angulo = ultAng;
    op.centrado = false;
    op.mascara = 0;
  } else {
    op.visto = false;
    op.centrado = false;
    op.mascara = 0;
    op.confianca = 0;
    op.existe = 0;
    op.busca_ganho = 0;                                // a fusão simples não tem mapa, então a busca usa o último lado
  }
  if (!op.mascara) op.dist_mm = -1;
  op.lado_ultimo = ultLado;
  op.ms_sem_ver  = jaViu ? (op.mascara ? 0 : agora - tVisto) : 0xFFFFFFFF;
}

// ==================== fusão com o rastreador bayesiano (FUSAO_BAYES 1) ====================
// O passo do mapa roda na tarefa dos sensores (passoRastreador); aqui, no núcleo 1, entra só a
// leitura da última estimativa pronta.
static void fundeBayes(Oponente& op) {
  uint32_t agora = millis();
  Estimativa e = rastreador_estimativa();        // última estimativa pronta (calculada no outro núcleo)
  bool existe  = e.existe >= FUSAO_EXISTE_MIN;   // oponente de verdade, e não leitura falsa nem arena vazia
  if (e.mascara && existe) { tVisto = agora; jaViu = true; }
  op.angulo    = e.angulo;
  op.x         = e.x;
  op.y         = e.y;
  op.sigma     = e.sigma;
  op.existe    = e.existe;
  op.busca_ang   = e.buscaAng;
  op.busca_ganho = e.buscaGanho;
  op.confianca = e.existe * e.massa;
  op.mascara   = existe ? e.mascara : 0;
  op.dist_mm   = existe ? e.distSensor : -1;
  op.visto     = jaViu && existe && e.massa >= FUSAO_MASSA_MIN && agora - tVisto <= perdeMs;
  bool centroOk = idxCentro >= 0 && ok[idxCentro];
  bool centroVe = centroOk ? (op.mascara & (1u << idxCentro)) != 0 : true;
  op.centrado  = op.visto && centroVe && fabsf(e.angulo) <= CENTRO_DEG;
  // O lado para procurar é onde o mapa diz que ele está, mesmo sem ver. Bem na frente
  // (menos de 3 graus) qualquer lado serve e o anterior fica, para não ficar trocando.
  // Com a arena vazia (existe baixo) o lado também não muda.
  if (jaViu && e.existe >= 0.3f && fabsf(e.angulo) >= 3.0f) ultLado = e.angulo > 0 ? 1 : -1;
  else if (jaViu && ultLado == 0) ultLado = 1;
  op.lado_ultimo = ultLado;
  op.ms_sem_ver  = jaViu ? (e.mascara ? 0 : agora - tVisto) : 0xFFFFFFFF;
  // Velocidade com que o oponente se aproxima (mm/s): queda da distância até o centro dele a
  // cada passo do mapa, menos a parte que vem do avanço do próprio robô, com filtro de 1a ordem.
  // Colado no oponente a distância não muda e a conta só junta ruído, então ela fica em zero.
  if (e.passo != passoAnt) {
    float dt = (agora - tRhoAnt) * 0.001f;
    if (op.visto && passoAnt != 0 && dt > 0.005f && dt < 0.2f && e.rho > 260) {
      float v = -(e.rho - rhoAnt) / dt - cinematica_v_mms() * cosf(e.angulo * (float)DEG_TO_RAD);
      v = constrain(v, -1500.0f, 1500.0f);
      aprox += 0.25f * (v - aprox);
    } else if (!op.visto || e.rho <= 260) aprox = 0;
    passoAnt = e.passo; rhoAnt = e.rho; tRhoAnt = agora;
  }
  op.aproximacao = aprox;
}

bool oponente_bayes() { return usaBayes; }

// escolha da fusão; a percepcao_atualiza chama esta função em toda volta do loop
void oponente_funde(Oponente& op) {
  imprimeAvisos();
  if (usaBayes) fundeBayes(op);
  else          { fundeSimples(op); op.aproximacao = 0; }
}
