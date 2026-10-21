// teste_wifi.cpp
// Teste do servidor WiFi do telemetria.cpp (USAR_WIFI 1), no computador. O WiFi.h de mentira
// (teste/unidade/stub_wifi) usa sockets de verdade, então o código do firmware conversa aqui com
// um cliente TCP real: /ping, /cmd, OPTIONS, o /stream em Server-Sent Events, a rede cheia (linha
// de dados descartada, linha de texto guardada e entregue depois, quadros sempre inteiros) e o
// desligamento do rádio quando a luta começa sem nenhum painel conectado.
// Uso (uma linha só):
//   g++ -std=c++11 -Wall -Wextra -DUSAR_WIFI=1 -I teste/unidade/stub_wifi -I teste/unidade/stub -I firmware/sumo_esp32 teste/unidade/teste_wifi.cpp firmware/sumo_esp32/telemetria.cpp -o t
//   ./t
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "WiFi.h"
#include "ESPmDNS.h"
#include "telemetria.h"
#include "estados.h"

#if !USAR_WIFI
#error "este teste precisa de -DUSAR_WIFI=1"
#endif

uint32_t relogio_us = 1000000;
int g_envio_max = 0;                          // limite de bytes por send() do firmware (lwip/sockets.h de mentira)
WiFiFalso WiFi;
MDNSFalso MDNS;
static Estado estadoFalso = ESPERA;
Estado estado_atual() { return estadoFalso; }

// O que sai pela "USB" (Serial de mentira) fica guardado para o teste conferir.
static std::string usb;
size_t SerialFalso::write(uint8_t c) { usb.push_back((char)c); return 1; }
SerialFalso Serial;

static int falhas = 0;
static void confere(bool ok, const char* oque, double valor = 0) {
  printf("%s  %-72s %9.1f\n", ok ? "ok   " : "FALHA", oque, valor);
  if (!ok) falhas++;
}

// Avança o relógio em passos de 1 ms, atendendo a rede a cada passo.
static void bombeia(int ms) { for (int i = 0; i < ms; i++) { relogio_us += 1000; telemetria_passo(); } }

// Cliente TCP do teste (o "navegador").
struct Cliente {
  int fd = -1;
  std::string rx;
  bool abre() {
    fd = socket(AF_INET, SOCK_STREAM, 0);
    int pequeno = 2048; setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &pequeno, sizeof(pequeno));
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_LOOPBACK); a.sin_port = htons(8080);
    if (connect(fd, (sockaddr*)&a, sizeof(a)) != 0) { perror("connect"); close(fd); fd = -1; return false; }
    return true;
  }
  void manda(const std::string& t) { if (send(fd, t.data(), t.size(), MSG_NOSIGNAL) < 0) perror("send"); }
  void le() {
    char b[4096];
    for (;;) { ssize_t r = recv(fd, b, sizeof(b), MSG_DONTWAIT); if (r <= 0) break; rx.append(b, (size_t)r); }
  }
  bool fechado() { char c; return recv(fd, &c, 1, MSG_PEEK | MSG_DONTWAIT) == 0; }
  void fecha() { if (fd >= 0) close(fd); fd = -1; }
  bool tem(const char* t) const { return rx.find(t) != std::string::npos; }
};

// Lê tudo o que o robô mandou até a rede ficar quieta.
static void drena(Cliente& c, int ms = 400) { for (int i = 0; i < ms; i++) { bombeia(1); c.le(); } }

static std::string lidoDoAnel() { std::string s; int ch; while ((ch = entrada_le()) >= 0) s.push_back((char)ch); return s; }

// Separa os eventos do /stream (depois do "retry: 1000") e confere o formato de cada um.
static bool quadrosInteiros(const std::string& rx, std::vector<std::string>& linhas) {
  size_t ini = rx.find("retry: 1000\n\n");
  if (ini == std::string::npos) return false;
  size_t pos = ini + strlen("retry: 1000\n\n");
  while (pos < rx.size()) {
    size_t fim = rx.find("\n\n", pos);
    if (fim == std::string::npos) return pos == rx.size();      // sobrou só a ponta do último quadro, ainda por chegar
    std::string ev = rx.substr(pos, fim - pos);
    if (ev.compare(0, 6, "data: ") != 0) return false;
    if (ev.find("data: ", 6) != std::string::npos) return false; // dois quadros misturados
    if (ev.find('\n') != std::string::npos) return false;
    linhas.push_back(ev.substr(6));
    pos = fim + 2;
  }
  return true;
}

int main() {
  telemetria_init();
  confere(telemetria_wifi_ligado(), "telemetria_init liga o WiFi");
  confere(usb.find("conecte na rede \"SUMO-ROBO\"") != std::string::npos, "USB mostra a rede e o endereco");

  // ---- /ping e cabeçalhos CORS (o painel abre do disco, origem diferente do robô)
  Cliente c;
  c.abre(); c.manda("GET /ping HTTP/1.1\r\nHost: x\r\n\r\n"); drena(c, 40);
  confere(c.tem("200 OK") && c.tem("pong"), "/ping responde pong");
  confere(c.tem("Access-Control-Allow-Origin: *"), "/ping libera o CORS");
  confere(c.fechado(), "/ping fecha a conexao (Connection: close)");
  c.fecha();

  // ---- /cmd entra no anel e sai pela entrada_le() como texto digitado
  c = Cliente(); c.abre(); c.manda("GET /cmd?c=mov%20100%200%20250%201 HTTP/1.1\r\n\r\n"); drena(c, 40);
  confere(c.tem("200 OK"), "/cmd responde 200");
  confere(lidoDoAnel() == "mov 100 0 250 1\n", "/cmd decodifica %20 e entrega o comando inteiro");
  c.fecha();
  c = Cliente(); c.abre(); c.manda("GET /cmd?c=pwm+30+30+250&x=1 HTTP/1.1\r\n\r\n"); drena(c, 40);
  confere(lidoDoAnel() == "pwm 30 30 250\n", "/cmd aceita + como espaco e para no &");
  c.fecha();
  c = Cliente(); c.abre(); c.manda("GET /cmd?c= HTTP/1.1\r\n\r\n"); drena(c, 40);
  confere(lidoDoAnel() == "\n", "/cmd vazio vale um ENTER");
  c.fecha();

  // ---- OPTIONS (consulta prévia do navegador), caminho inexistente e método errado
  c = Cliente(); c.abre(); c.manda("OPTIONS /cmd HTTP/1.1\r\nOrigin: null\r\n\r\n"); drena(c, 40);
  confere(c.tem("204") && c.tem("Access-Control-Allow-Private-Network: true"), "OPTIONS responde 204 com permissao da rede local");
  c.fecha();
  c = Cliente(); c.abre(); c.manda("GET /nada HTTP/1.1\r\n\r\n"); drena(c, 40);
  confere(c.tem("404"), "caminho desconhecido: 404");
  c.fecha();
  c = Cliente(); c.abre(); c.manda("POST /cmd HTTP/1.1\r\n\r\n"); drena(c, 40);
  confere(c.tem("405"), "POST: 405");
  c.fecha();
  confere(lidoDoAnel().empty(), "as requisições recusadas não deixam comando no anel");

  // ---- /stream: cada linha do Log vira um evento "data: ..."
  Cliente p; p.abre(); p.manda("GET /stream HTTP/1.1\r\nAccept: text/event-stream\r\n\r\n"); drena(p, 40);
  confere(p.tem("text/event-stream") && p.tem("retry: 1000"), "/stream abre o fluxo de eventos");
  confere(telemetria_paineis() == 1, "um painel conectado", telemetria_paineis());
  Log.println("D,1,2,3"); Log.println("# ola"); drena(p, 20);
  confere(p.tem("data: D,1,2,3\n\n") && p.tem("data: # ola\n\n"), "as linhas do Log chegam como eventos SSE");

  // ---- rede cheia: o painel não lê, o robô nunca espera
  {
    uint32_t desc0 = telemetria_descartes_wifi();
    srand(7);
    int enviadas = 0;
    for (int i = 0; i < 4000 && telemetria_descartes_wifi() == desc0; i++) {
      char l[500]; int n = 20 + rand() % 400;
      int k = snprintf(l, sizeof(l), "D,%d,", i);
      for (; k < n; k++) l[k] = 'a' + (k % 26);
      l[k] = 0;
      Log.println(l); enviadas++;
    }
    confere(telemetria_descartes_wifi() > desc0, "rede cheia: linha de dados descartada em vez de travar", telemetria_descartes_wifi() - desc0);
    // o socket está cheio agora: uma linha de texto não pode se perder
    Log.println("# importante");
    for (int i = 0; i < 50; i++) { Log.println("D,999999,descartavel"); }   // enquanto deve o resto, o painel não recebe linha nova no meio
    drena(p, 800);
    std::vector<std::string> linhas;
    confere(quadrosInteiros(p.rx, linhas), "depois de encher e drenar, todos os quadros estão inteiros");
    bool achou = false; int ant = -1; bool cresce = true;
    for (size_t i = 0; i < linhas.size(); i++) {
      if (linhas[i] == "# importante") achou = true;
      if (linhas[i].compare(0, 2, "D,") == 0 && linhas[i] != "D,1,2,3") { int n = atoi(linhas[i].c_str() + 2); if (n != 999999) { if (n <= ant) cresce = false; ant = n; } }
    }
    confere(achou, "a linha de texto que não coube na hora foi entregue depois");
    confere(cresce, "as linhas de dados chegam em ordem (só faltam as descartadas)");
    (void)enviadas;
    // a rede se recupera: a linha seguinte chega normal
    Log.println("# depois"); drena(p, 40);
    confere(p.tem("data: # depois\n\n"), "passada a rede cheia, o fluxo volta ao normal");
  }

  // ---- envio parcial: a rede aceita só alguns bytes por vez e o resto da linha sai nas voltas seguintes
  {
    p.rx.clear();
    p.manda("");                                         // nada a enviar, só garante o socket vivo
    g_envio_max = 7;
    Log.println("# parcial um dois tres quatro cinco seis sete");
    drena(p, 200);
    confere(p.tem("data: # parcial um dois tres quatro cinco seis sete\n\n"), "envio parcial: a linha de texto chega inteira, 7 bytes por volta");
    Log.println("D,77,parcial"); drena(p, 100);
    Log.println("# fim do parcial"); drena(p, 100);
    g_envio_max = 0;
    std::vector<std::string> linhas;
    confere(quadrosInteiros("retry: 1000\n\n" + p.rx, linhas), "envio parcial: nenhum quadro cortado ou misturado");
    confere(p.tem("data: # fim do parcial\n\n"), "envio parcial: o fluxo continua depois");
  }

  // ---- desligar o rádio fecha tudo
  usb.clear();
  telemetria_wifi_desliga("teste");
  confere(!telemetria_wifi_ligado(), "wifi off desliga o radio");
  confere(usb.find("WiFi desligado: teste") != std::string::npos, "o motivo do desligamento aparece na USB");
  drena(p, 10);
  confere(p.fechado(), "o painel conectado é desconectado");
  p.fecha();
  Log.println("D,5,5,5");                                  // sem rádio, o Log continua só na USB
  confere(usb.find("D,5,5,5") != std::string::npos, "sem rádio o Log segue funcionando pela USB");

  // ---- WIFI_DESLIGA_SEM_PAINEL: a luta começa e ninguém está conectado
  usb.clear(); estadoFalso = ESPERA;
  telemetria_init();
  bombeia(10);
  confere(telemetria_wifi_ligado(), "na ESPERA o radio fica ligado mesmo sem painel");
  estadoFalso = CONTAGEM; bombeia(5);
  confere(!telemetria_wifi_ligado(), "luta iniciada sem painel: radio desligado sozinho");
  confere(usb.find("luta iniciada sem painel conectado") != std::string::npos, "o desligamento automatico explica o motivo");

  // ---- com painel conectado, a luta não desliga o rádio; o painel sair desliga
  usb.clear(); estadoFalso = ESPERA;
  telemetria_init();
  p = Cliente(); p.abre(); p.manda("GET /stream HTTP/1.1\r\n\r\n"); drena(p, 40);
  confere(telemetria_paineis() == 1, "painel reconectado depois de religar o radio", telemetria_paineis());
  estadoFalso = CONTAGEM; bombeia(20);
  confere(telemetria_wifi_ligado(), "luta com painel conectado: radio continua ligado");
  p.fecha(); bombeia(20);
  confere(!telemetria_wifi_ligado(), "o painel saiu durante a luta: radio desligado");

  printf("\n%s: %d falha(s)\n", falhas ? "ERRO" : "OK", falhas);
  return falhas ? 1 : 0;
}
