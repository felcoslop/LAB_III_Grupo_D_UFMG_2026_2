// oponente.cpp
// Módulo dos 5 sensores de oponente VL53L0X (placas CJVL53L0XV2 / GY-530).
// Ele cuida de três coisas: ligar os sensores e dar um endereço I2C para cada um, ler as
// distâncias sem travar o loop e juntar as leituras em um único oponente (a fusão).
// As funções públicas são chamadas pelo percepcao.cpp e pelo debug.cpp.
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
#include "motores.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <Preferences.h>
#include <math.h>

static VL53L0X  sensor[N_TOF];        // um objeto da biblioteca Pololu para cada sensor
static bool     ok[N_TOF];            // sensor respondendo
static int16_t  bruto[N_TOF], filtrado[N_TOF];   // leitura crua e leitura corrigida com mediana
static int16_t  hist[N_TOF][3];       // últimas 3 leituras de cada sensor, para a mediana
static uint8_t  hIdx[N_TOF];          // posição atual no histórico circular
static uint32_t tUlt[N_TOF];          // momento da última medida nova
static uint16_t seq[N_TOF];           // contador de medidas novas
static uint8_t  errI2C[N_TOF];        // erros de I2C seguidos (sensor que sumiu do barramento)
static int8_t   idxCentro = -1;       // índice do sensor central

// calibração: leitura corrigida = ganho * bruto + offset
static float    calGanho[N_TOF];
static float    calOffset[N_TOF];
static bool     calNvs[N_TOF];        // true quando a calibração veio do comando "cal" e está gravada
static Preferences prefs;             // acesso à memória NVS do ESP32

// estado da fusão
static float    ultAng = 0;
static int8_t   ultLado = 0;
static uint32_t tVisto = 0;
static bool     jaViu = false;

// dados para o rastreador: última medida corrigida de cada sensor, sem a mediana
// (-1 significa que mediu e não tem nada no alcance; -2 significa sem medida válida)
static int16_t  ultZ[N_TOF];
static int16_t  alcance[N_TOF];      // maior leitura que ainda fica dentro do raio TOF_ALCANCE_MM em volta do robô
static uint16_t seqUsado[N_TOF];     // última medida já entregue ao rastreador
static bool     usaBayes = false;    // true quando o rastreador conseguiu memória e está ativo
static uint32_t tPasso = 0;          // momento do último passo do rastreador

static const uint32_t PERIODO_MS = TOF_TIMING_US / 1000;   // tempo entre medidas de cada sensor

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
  sensor[i].setMeasurementTimingBudget(TOF_TIMING_US);
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

// Inicialização dos ToF, chamada no percepcao_init.
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
                  ok[i] ? "OK" : "FALHOU (VCC/GND/SDA/SCL frouxo ou fio XSHUT deste sensor)");
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
    if (usaBayes) Log.printf("# rastreador bayesiano: oponente %d mm, mapa 120x38 pronto em %lu ms\n",
                             OPONENTE_LADO_MM, (unsigned long)(millis() - t0));
    else          Log.println(F("! rastreador: sem memoria, usando a fusao simples"));
  } else rastreador_zera();
#endif
  return nOk;
}

// nova tentativa nos sensores que falharam, chamada pela percepcao_manutencao com o robô parado
uint8_t tof_reinicia_falhos() {
  uint8_t recuperados = 0;
  for (uint8_t i = 0; i < N_TOF; i++) {
    if (ok[i]) continue;
    if (iniciaUm(i)) {
      ok[i] = true; zeraLeitura(i); recuperados++;
      Log.printf("# %s recuperado (0x%02X)\n", TOF[i].nome, TOF[i].endereco);
    }
  }
  return recuperados;
}

// mediana de 3 valores com três trocas, sem ordenar vetor
static int16_t mediana3(int16_t a, int16_t b, int16_t c) {
  if (a > b) { int16_t t = a; a = b; b = t; }
  if (b > c) { int16_t t = b; b = c; c = t; }
  if (a > b) { int16_t t = a; a = b; b = t; }
  return b;
}

// Leitura dos ToF, chamada em toda volta do loop pela percepcao_atualiza.
void tof_atualiza() {
  uint32_t agora = millis();
  for (uint8_t i = 0; i < N_TOF; i++) {
    if (!ok[i]) continue;
    uint32_t desde = agora - tUlt[i];

    // leitura escalonada: o sensor só é consultado perto da hora de ter medida nova,
    // o que economiza o barramento I2C e sobra tempo para borda e motores
    if (desde + 3 < PERIODO_MS) continue;

    uint8_t st = sensor[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if (sensor[i].last_status != 0) {               // o sensor não respondeu no I2C
      if (errI2C[i] < 255) errI2C[i]++;
      if (desde > TOF_TIMEOUT_MS) { bruto[i] = -1; filtrado[i] = -1; ultZ[i] = -2; }
      // O sensor só vira FALHOU com erro repetido de verdade. Uma pausa longa do loop
      // (por exemplo, enviando o painel pelo WiFi) não derruba um sensor bom.
      if (errI2C[i] >= 10 && desde > TOF_PERDIDO_MS) {
        ok[i] = false;
        xshutDesliga(i);                            // o sensor volta desligado para o 0x29 e pode ser religado depois
        zeraLeitura(i);
        Log.printf("! %s parou de responder\n", TOF[i].nome);
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
    seq[i]++;

    int16_t c = -1;
    if (r > 0 && r < 8000) {                                // 8190 e 8191 significam nada à frente
      bruto[i] = r;
      c = (int16_t)lroundf(calGanho[i] * r + calOffset[i]); // aplicação da calibração
      ultZ[i] = c > alcance[i] ? -1 : (c < TOF_MIN_MM ? -2 : c);
    }
    else { bruto[i] = -1; ultZ[i] = r == 0 ? -2 : -1; }

    // histórico circular de 3 leituras: a mediana some com uma leitura falsa isolada
    hist[i][hIdx[i]] = c;
    hIdx[i] = (hIdx[i] + 1) % 3;
    int16_t m = mediana3(hist[i][0], hist[i][1], hist[i][2]);
    filtrado[i] = (m >= TOF_MIN_MM && m <= alcance[i]) ? m : -1;
  }
}

bool     tof_ok(uint8_t i)    { return i < N_TOF && ok[i]; }
int16_t  tof_mm(uint8_t i)    { return i < N_TOF ? filtrado[i] : -1; }
int16_t  tof_bruto(uint8_t i) { return i < N_TOF ? bruto[i] : -1; }
int16_t  tof_cru(uint8_t i)   { return i < N_TOF ? ultZ[i] : -2; }
uint16_t tof_seq(uint8_t i)   { return i < N_TOF ? seq[i] : 0; }
int8_t   tof_centro()         { return idxCentro; }

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

// ==================== fusão simples (FUSAO_BAYES 0) ====================
// Cada leitura vira um ponto (x,y) no referencial do robô. Leituras do mesmo objeto
// (distância parecida e sensores vizinhos) viram um ponto médio, com peso maior para quem
// está mais perto, e esse ponto dá um ângulo. Como a origem é o centro de giro, esse ângulo
// é exatamente o quanto o robô precisa girar.
static void fundeSimples(Oponente& op) {
  uint32_t agora = millis();
  float px[N_TOF], py[N_TOF];
  int8_t perto = -1;
  int16_t dPerto = 32767;

  // primeiro laço: converte as leituras em pontos e acha o sensor que vê mais perto
  for (uint8_t i = 0; i < N_TOF; i++) {
    int16_t d = filtrado[i];
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
      if (filtrado[i] <= 0) continue;
      // mesmo objeto: distância parecida e sensores vizinhos (frente e lateral ficam separadas)
      bool distParecida = abs(filtrado[i] - dPerto) <= FUSAO_CLUSTER_MM;
      bool vizinho      = abs(TOF[i].ang - TOF[perto].ang) <= FUSAO_CLUSTER_DEG;
      if (distParecida && vizinho) {
        float w = 1.0f / (filtrado[i] + 30.0f);        // quem está perto pesa mais
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
  } else if (jaViu && agora - tVisto <= FUSAO_PERDE_MS) {
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
// A cada medida nova (20 ms) o mapa de probabilidade do rastreador.cpp recebe todos os
// sensores: quem vê informa "está a z mm" e quem não vê informa "no meu cone não está".
static void fundeBayes(Oponente& op) {
  uint32_t agora = millis();
  if (agora - tPasso >= PERIODO_MS) {
    float dt = (agora - tPasso) * 0.001f;
    tPasso = agora;
    int16_t z[N_TOF];
    bool novo[N_TOF];
    // o laço separa as medidas novas (seq mudou) das repetidas, que não entram de novo no filtro
    for (uint8_t i = 0; i < N_TOF; i++) {
      novo[i] = ok[i] && seq[i] != seqUsado[i] && ultZ[i] != -2;
      seqUsado[i] = seq[i];
      z[i] = (ok[i] && ultZ[i] > 0) ? ultZ[i] : -1;
    }
#if USAR_MOTORES
    // com motores, o giro e o avanço saem dos comandos dados (motores.cpp) e o mapa gira junto
    float esq = motores_esq(), dir = motores_dir();
    rastreador_passo(z, novo, GIRO_MAX_DPS * (dir - esq) / 510.0f, true,
                     ROBO_VEL_MAX_MMS * (esq + dir) / 510.0f, dt);
#else
    // na bancada o robô é girado na mão, sem aviso, e esse giro entra como incerteza
    rastreador_passo(z, novo, 0, false, 0, dt);
#endif
  }

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
  op.visto     = jaViu && existe && e.massa >= FUSAO_MASSA_MIN && agora - tVisto <= FUSAO_PERDE_MS;
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
}

bool oponente_bayes() { return usaBayes; }

// escolha da fusão; a percepcao_atualiza chama esta função em toda volta do loop
void oponente_funde(Oponente& op) {
  if (usaBayes) fundeBayes(op);
  else          fundeSimples(op);
}
