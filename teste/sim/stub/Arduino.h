// Arduino.h de mentira do simulador de luta (teste/sim). Ele cobre a parte da API do ESP32 que
// o firmware usa, e cada função conversa com o mundo simulado (hal.cpp): o relógio é o tempo da
// simulação, o analogReadMilliVolts devolve o sensor de borda sobre o dojô simulado, o
// digitalRead devolve a chave DIP e o botão, e o ledcWrite entrega o PWM para os motores da
// física. Não existe FreeRTOS: sem ARDUINO_ARCH_ESP32, o firmware roda tudo no próprio loop.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <math.h>
#include <cmath>
#include <algorithm>

typedef bool boolean;
typedef uint8_t byte;

#define PI          3.1415926535897932384626433832795
#define HALF_PI     1.5707963267948966192313216916398
#define TWO_PI      6.283185307179586476925286766559
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG  57.295779513082320876798154814105
#define HIGH 1
#define LOW  0
#define INPUT          0x01
#define OUTPUT         0x03
#define PULLUP         0x04
#define INPUT_PULLUP   0x05
#define PULLDOWN       0x08
#define INPUT_PULLDOWN 0x09
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define F(texto) (texto)
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

using std::min;
using std::max;

// relógio e pinos (implementados no hal.cpp)
uint32_t millis();
uint32_t micros();
void     delay(uint32_t ms);
void     delayMicroseconds(uint32_t us);
void     yield();
void     pinMode(uint8_t pino, uint8_t modo);
void     digitalWrite(uint8_t pino, uint8_t valor);
int      digitalRead(uint8_t pino);
void     analogReadResolution(uint8_t bits);
uint32_t analogReadMilliVolts(uint8_t pino);
uint16_t analogRead(uint8_t pino);

// PWM do LEDC (as duas linhas do core)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
bool ledcWrite(uint8_t pin, uint32_t duty);
#else
uint32_t ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
void ledcAttachPin(uint8_t pin, uint8_t channel);
void ledcWrite(uint8_t channel, uint32_t duty);
#endif

// Print do Arduino: o printf monta o texto e manda caractere por caractere para o write
class Print {
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
  size_t write(const char* s) { return write((const uint8_t*)s, strlen(s)); }
  size_t print(const char* s)   { return write((const uint8_t*)s, strlen(s)); }
  size_t print(char c)          { return write((uint8_t)c); }
  size_t print(int v)           { char b[16]; snprintf(b, sizeof(b), "%d", v); return print(b); }
  size_t println(const char* s) { size_t n = print(s); return n + write('\n'); }
  size_t println()              { return write('\n'); }
  size_t printf(const char* f, ...) __attribute__((format(printf, 2, 3))) {
    char buf[1024]; va_list a; va_start(a, f); int n = vsnprintf(buf, sizeof(buf), f, a); va_end(a);
    return n > 0 ? write((const uint8_t*)buf, strlen(buf)) : 0;
  }
};

// Serial: a saída vai para o stdout só quando o simulador pede (opção -v)
class HardwareSerial : public Print {
public:
  using Print::write;
  void   begin(unsigned long) {}
  void   setTxBufferSize(size_t) {}
  int    available();
  int    read();
  int    availableForWrite() { return 4096; }
  size_t write(uint8_t c) override;
};
extern HardwareSerial Serial;
