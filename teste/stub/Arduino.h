// Arduino.h de mentira para compilar módulos do firmware no computador (testes do CI).
// Só tem o que os módulos testados usam; o relógio é controlado pelo próprio teste.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <cmath>
extern uint32_t relogio_us;                 // o teste avança este valor
inline uint32_t micros() { return relogio_us; }
inline uint32_t millis() { return relogio_us / 1000; }
#define PI          3.1415926535897932384626433832795
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG  57.295779513082320876798154814105
#define HIGH 1
#define LOW  0
#define OUTPUT 1
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define F(texto) (texto)
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}

// Print simplificado: o printf monta o texto e manda caractere por caractere para o write.
class Print {
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
  size_t print(const char* s)   { return write((const uint8_t*)s, strlen(s)); }
  size_t println(const char* s) { size_t n = print(s); return n + write('\n'); }
  size_t printf(const char* f, ...) {
    char buf[256]; va_list a; va_start(a, f); int n = vsnprintf(buf, sizeof(buf), f, a); va_end(a);
    return n > 0 ? write((const uint8_t*)buf, strlen(buf)) : 0;
  }
};
