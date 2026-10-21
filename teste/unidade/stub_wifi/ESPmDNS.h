// ESPmDNS.h de mentira (teste no computador): o mDNS não faz nada.
#pragma once
struct MDNSFalso {
  bool begin(const char*) { return true; }
  void addService(const char*, const char*, int) {}
  void end() {}
};
extern MDNSFalso MDNS;
