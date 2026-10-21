// Preferences.h de mentira para os testes no computador: a memória não volátil do ESP32 vira um mapa
// em RAM que continua valendo entre duas chamadas de begin(), como a flash entre dois boots.
#pragma once
#include <map>
#include <string>
#include "Arduino.h"

class Preferences {
  std::string ns;
  static std::map<std::string, uint16_t>& db() { static std::map<std::string, uint16_t> d; return d; }
public:
  bool begin(const char* nome, bool = false) { ns = nome; return true; }
  void end() {}
  bool isKey(const char* k) { return db().count(ns + "/" + k) > 0; }
  uint16_t getUShort(const char* k, uint16_t padrao = 0) { auto it = db().find(ns + "/" + k); return it == db().end() ? padrao : it->second; }
  size_t putUShort(const char* k, uint16_t v) { db()[ns + "/" + k] = v; return 2; }
  bool remove(const char* k) { return db().erase(ns + "/" + k) > 0; }
  bool clear() {
    std::string pre = ns + "/";
    for (auto it = db().begin(); it != db().end();) { if (it->first.compare(0, pre.size(), pre) == 0) it = db().erase(it); else ++it; }
    return true;
  }
};
