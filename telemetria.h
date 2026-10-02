// telemetria.h
// Saída de texto (USB e WiFi) e entrada de comandos (USB e WiFi).
// O objeto Log substitui Serial.print, Serial.printf e Serial.println no projeto todo: o texto
// vai para a USB e, quando existe painel conectado pelo WiFi, vai para o painel também.
// A função entrada_le() devolve o próximo caractere digitado, venha ele da USB ou do painel,
// e devolve -1 quando não tem nada.
//
// Com USAR_WIFI 1 no config.h, o ESP32 cria a rede "SUMO-ROBO" (ou entra no roteador, quando
// configurado) e atende três endereços:
//   http://192.168.4.1                abre o painel guardado no próprio ESP32 (sem internet)
//   http://192.168.4.1/stream         dados em tempo real (Server-Sent Events)
//   http://192.168.4.1/cmd?c=lista    envia um comando
// Nenhuma parte deste módulo bloqueia o robô; quando o WiFi engasga, o pacote é descartado.
#pragma once
#include <Arduino.h>

// Saida herda de Print, então Log aceita print, println e printf como o Serial.
class Saida : public Print {
public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t n) override;
};
extern Saida Log;

void telemetria_init();
void telemetria_passo();          // uma chamada por volta do loop; a rede é atendida sem bloquear
bool telemetria_novo_painel();    // true uma única vez logo depois que um painel conecta
uint8_t telemetria_paineis();     // quantos painéis estão recebendo dados
int  entrada_le();                // próximo caractere de comando (-1 quando não tem nada)
uint32_t telemetria_descartes_usb(); // linhas de dados não enviadas pela USB por causa da serial cheia
