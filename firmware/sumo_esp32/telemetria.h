// telemetria.h
// Saída de texto e entrada de comandos pela USB e, com USAR_WIFI 1, também pelo WiFi.
// O objeto Log substitui Serial.print, Serial.printf e Serial.println no projeto todo: o texto
// sai sempre em linhas inteiras, o que deixa o painel (painel_sumo.html) ler cada linha sem
// pedaços misturados. A função entrada_le() devolve o próximo caractere digitado no Monitor
// Serial ou enviado pelo painel (pela USB ou pelo WiFi) e devolve -1 quando não tem nada.
//
// Com USAR_WIFI 0 (padrão) nenhuma parte do rádio é compilada e as funções do WiFi abaixo não
// fazem nada. Com USAR_WIFI 1, o ESP32 cria a rede WIFI_AP_SSID (ou entra no roteador) e atende
// três endereços, sem precisar de internet:
//   http://192.168.4.1/stream         dados em tempo real (Server-Sent Events), as mesmas linhas da USB
//   http://192.168.4.1/cmd?c=lista    envia um comando, como se fosse digitado
//   http://192.168.4.1/ping           resposta curta para o painel conferir se o robô está na rede
// Nenhuma parte deste módulo bloqueia o robô; quando o WiFi engasga, a linha de dados é descartada.
#pragma once
#include <Arduino.h>

// Saida herda de Print, então Log aceita print, println e printf como o Serial.
class Saida : public Print {
public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t n) override;
};
extern Saida Log;

int  entrada_le();                // próximo caractere de comando (-1 quando não tem nada)
uint32_t telemetria_descartes_usb(); // linhas de dados não enviadas pela USB por causa da serial cheia

// WiFi (USAR_WIFI 1). Com USAR_WIFI 0 as funções existem e não fazem nada.
void     telemetria_init();       // liga a rede e o servidor (chamada uma vez no setup)
void     telemetria_passo();      // uma chamada por volta do loop; atende a rede sem bloquear
uint8_t  telemetria_paineis();    // quantos painéis estão recebendo dados pelo WiFi
bool     telemetria_wifi_ligado();// true enquanto o rádio está ligado
void     telemetria_wifi_desliga(const char* motivo);   // desliga o rádio (comando "wifi off" e partida sem painel)
void     telemetria_wifi_info();  // linha do terminal com rede, endereço, painéis e linhas descartadas
uint32_t telemetria_descartes_wifi(); // linhas de dados não enviadas pelo WiFi por causa da rede cheia
