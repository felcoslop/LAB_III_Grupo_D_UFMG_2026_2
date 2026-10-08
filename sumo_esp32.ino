// sumo_esp32.ino
// Arquivo principal do firmware do robô sumô 1 kg do Grupo D (Laboratório de Sistemas III, UFMG).
// O setup inicializa cada módulo uma única vez e o loop chama os módulos em sequência.
// Nenhum módulo usa delay dentro do loop: cada um faz uma parte pequena do trabalho e
// devolve o controle, assim a borda e os motores nunca ficam parados esperando outra tarefa.
// O robô é autônomo e o firmware não liga o rádio (sem WiFi e sem Bluetooth).
//
// Configuração usada na Arduino IDE: placa "ESP32 Dev Module", biblioteca "VL53L0X" da
// Pololu (versão 1.3.1) e monitor serial em 115200 com "Nova linha". O Partition Scheme
// padrão já comporta o firmware.
//
// Fluxo de dados: sensores (percepcao) geram a struct P, a máquina de estados (estados)
// lê P e manda nos motores, e o debug mostra tudo pela USB no terminal e no painel.

#include "config.h"
#include "telemetria.h"
#include "percepcao.h"
#include "motores.h"
#include "estados.h"
#include "debug.h"

void setup() {
  motores_init();          // os motores são configurados primeiro para o robô nascer parado
  Serial.setTxBufferSize(4096);   // buffer de 4 KB na serial: um print grande não segura o loop
  Serial.begin(SERIAL_BAUD);
  delay(300);
  Log.println(F("\n# ===== ROBO SUMO | ESP32 ====="));
  percepcao_init();        // inicialização do I2C, dos ToF, da borda e do botão (percepcao.cpp)
  estados_init();          // máquina de estados começa em ESPERA (estados.cpp)
  debug_init();            // a lista de comandos aparece no terminal (debug.cpp)
}

void loop() {
  percepcao_atualiza();    // passo 1: a percepção lê todos os sensores e monta a struct P
  estados_passo();         // passo 2: a máquina de estados decide e comanda os motores
  if (estado_atual() == ESPERA) percepcao_manutencao();   // com o robô parado, o código tenta religar sensor que caiu
  debug_passo();           // passo 3: o debug trata os comandos e envia os dados para o painel pela USB
}
