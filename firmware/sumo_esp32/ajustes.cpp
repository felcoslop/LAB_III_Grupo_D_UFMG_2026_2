// ajustes.cpp
// Leitura e gravação dos ajustes do painel na NVS (explicação no ajustes.h).
#include "ajustes.h"
#include <Preferences.h>
#include <strings.h>   // strcasecmp

static const char* NS = "ajustes";

static Ajustes fabrica() { return Ajustes{QTD_TOF, AUTO_START != 0, false, false, false}; }

void ajustes_carrega() {
  Ajustes& a = ajustes();
  a = fabrica();
  Preferences pr;
  if (!pr.begin(NS, true)) return;
  uint16_t t = pr.getUShort("tof", a.tof);
  if (t == 5 || t == 7) a.tof = (uint8_t)t;
  if (pr.isKey("auto")) a.autonomo = pr.getUShort("auto", 0) != 0;
  if (pr.isKey("blat")) a.borda_lateral = pr.getUShort("blat", 0) != 0;
  if (pr.isKey("alin")) a.alinha_contagem = pr.getUShort("alin", 0) != 0;
  if (pr.isKey("wifi")) a.wifi_luta = pr.getUShort("wifi", 0) != 0;
  pr.end();
}

int8_t ajustes_define(const char* nome, int valor) {
  Ajustes& a = ajustes();
  const char* chave = nullptr;
  int8_t res = AJUSTE_OK;
  if (!strcasecmp(nome, "tof")) {
    if (valor != 5 && valor != 7) return AJUSTE_INVALIDO;
    chave = "tof";
    if (a.tof != valor) res = AJUSTE_REINICIA;   // o resto do firmware leu a quantidade no boot; só aplica depois de reiniciar
  } else {
    if (valor != 0 && valor != 1) return AJUSTE_INVALIDO;
    if      (!strcasecmp(nome, "auto"))   { chave = "auto"; a.autonomo = valor; }
    else if (!strcasecmp(nome, "linha"))  { chave = "blat"; a.borda_lateral = valor; }
    else if (!strcasecmp(nome, "alinha")) { chave = "alin"; a.alinha_contagem = valor; }
    else if (!strcasecmp(nome, "wifi"))   { chave = "wifi"; a.wifi_luta = valor; }
    else return AJUSTE_INVALIDO;
  }
  Preferences pr;
  if (pr.begin(NS, false)) { pr.putUShort(chave, (uint16_t)valor); pr.end(); }
  // com a quantidade de ToF ainda por reiniciar, o valor em uso (a.tof) continua o antigo até o próximo boot
  return res;
}

int8_t ajustes_padrao() {
  Preferences pr;
  if (pr.begin(NS, false)) { pr.clear(); pr.end(); }
  const uint8_t emUso = ajustes().tof;      // a quantidade de ToF em uso só muda no próximo boot
  ajustes() = fabrica();
  ajustes().tof = emUso;
  return emUso != fabrica().tof ? AJUSTE_REINICIA : AJUSTE_OK;
}

bool ajustes_gravado() {
  Preferences pr;
  if (!pr.begin(NS, true)) return false;
  bool g = pr.isKey("tof") || pr.isKey("auto") || pr.isKey("blat") || pr.isKey("alin") || pr.isKey("wifi");
  pr.end();
  return g;
}
