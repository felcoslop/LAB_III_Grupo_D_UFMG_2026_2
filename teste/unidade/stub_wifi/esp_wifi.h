// esp_wifi.h de mentira para compilar o telemetria.cpp (USAR_WIFI 1) no computador: só o que o firmware usa para fixar a
// largura do canal do ponto de acesso em 20 MHz. A chamada fica registrada para o teste conferir.
#pragma once
#include <stdint.h>

typedef enum { WIFI_IF_STA = 0, WIFI_IF_AP = 1 } wifi_interface_t;
typedef enum { WIFI_BW_HT20 = 1, WIFI_BW_HT40 = 2 } wifi_bandwidth_t;
typedef int esp_err_t;

extern int g_wifi_if_largura;        // interface da última chamada (-1 antes de qualquer chamada)
extern int g_wifi_largura;           // largura pedida na última chamada (-1 antes de qualquer chamada)
inline esp_err_t esp_wifi_set_bandwidth(wifi_interface_t i, wifi_bandwidth_t b) {
  g_wifi_if_largura = (int)i; g_wifi_largura = (int)b; return 0;
}
