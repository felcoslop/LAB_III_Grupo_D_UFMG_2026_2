// Preferences.h de mentira do simulador: a memória NVS vira um mapa em RAM, vazio no começo de
// cada luta (sem calibração gravada, como um ESP32 recém-gravado).
#pragma once
#include "Arduino.h"
#include <map>
#include <string>

class Preferences {
public:
  bool     begin(const char* ns, bool = false) { nome = ns; return true; }
  void     end() {}
  bool     isKey(const char* k) { return dados().count(chave(k)) > 0; }
  uint16_t getUShort(const char* k, uint16_t d = 0) { return isKey(k) ? (uint16_t)dados()[chave(k)] : d; }
  size_t   putUShort(const char* k, uint16_t v) { dados()[chave(k)] = v; return 2; }
  float    getFloat(const char* k, float d = 0) { return isKey(k) ? (float)dados()[chave(k)] : d; }
  size_t   putFloat(const char* k, float v) { dados()[chave(k)] = v; return 4; }
  uint8_t  getUChar(const char* k, uint8_t d = 0) { return isKey(k) ? (uint8_t)dados()[chave(k)] : d; }
  size_t   putUChar(const char* k, uint8_t v) { dados()[chave(k)] = v; return 1; }
  bool     remove(const char* k) { return dados().erase(chave(k)) > 0; }
  bool     clear() {
    for (auto it = dados().begin(); it != dados().end();) it = it->first.rfind(nome + "/", 0) == 0 ? dados().erase(it) : std::next(it);
    return true;
  }
private:
  std::string nome;
  std::string chave(const char* k) { return nome + "/" + k; }
  static std::map<std::string, double>& dados() { static std::map<std::string, double> m; return m; }
};
