// telemetria.h
// Saída de texto e entrada de comandos pela USB.
// O objeto Log substitui Serial.print, Serial.printf e Serial.println no projeto todo: o texto
// sai pela USB sempre em linhas inteiras, o que deixa o painel (painel_sumo.html) ler cada linha
// sem pedaços misturados. A função entrada_le() devolve o próximo caractere digitado no Monitor
// Serial ou enviado pelo painel, e devolve -1 quando não tem nada.
// O robô é autônomo e não usa rádio: nenhuma parte deste módulo liga WiFi ou Bluetooth.
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
