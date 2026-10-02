// telemetria.cpp
// WiFi feito só com as bibliotecas que já vêm no core do ESP32.
// A escolha foi Server-Sent Events em vez de WebSocket porque o navegador reconecta sozinho,
// nenhuma biblioteca extra precisa ser instalada na IDE e o código funciona igual nos cores
// 2.x e 3.x. Os comandos do painel chegam por /cmd, em uma requisição curta.
//
// Para manter o tempo real sem travar o robô, três cuidados foram tomados:
//   WiFi.setSleep(false) deixa o rádio sempre acordado (dormir soma uns 100 ms de atraso);
//   setNoDelay faz cada linha sair na hora, sem esperar para juntar pacotes;
//   o envio usa MSG_DONTWAIT, então com a rede cheia a linha é descartada em vez de segurar
//   o loop. A telemetria perde um quadro, mas o robô nunca espera.

#include "telemetria.h"
#include "config.h"
#include "estados.h"

Saida Log;

// Saída por linha inteira.
// O texto fica acumulado até o '\n' e só então sai inteiro pela USB e pelo WiFi. As linhas de
// dados (D, Q e M) são descartadas na USB quando o buffer de envio está cheio, já que a
// 115200 baud cabem só uns 11 KB/s. O buffer de 4 KB da serial é criado no sumo_esp32.ino,
// antes do Serial.begin.
static char     lbuf[1024];          // tamanho suficiente para a linha do mapa (até uns 800 caracteres)
static uint16_t lN = 0;              // quantidade de caracteres na linha atual
static uint32_t descartesUsb = 0;
static void enviaPaineis(const char* s, uint16_t n);

// Envio da linha acumulada. Linhas de texto comum sempre saem na USB; linhas de dados só saem
// quando cabem no buffer da serial. Para o painel a linha vai sempre.
static void fechaLinha() {
  bool dado = lN >= 2 && lbuf[1] == ',' && (lbuf[0] == 'D' || lbuf[0] == 'Q' || lbuf[0] == 'M');
  if (!dado || Serial.availableForWrite() > (int)lN + 2) {
    Serial.write((const uint8_t*)lbuf, lN);
    Serial.write('\n');
  } else descartesUsb++;
  enviaPaineis(lbuf, lN);
  lN = 0;
}

// Cada caractere escrito no Log passa por aqui (o print e o printf da classe Print chamam write).
size_t Saida::write(uint8_t c) {
  if (c == '\r') return 1;                       // o '\r' é ignorado, só o '\n' fecha a linha
  if (c == '\n') { fechaLinha(); return 1; }
  if (lN >= sizeof(lbuf) - 1) fechaLinha();      // linha grande demais: o que já existe é enviado
  lbuf[lN++] = c;
  return 1;
}
size_t Saida::write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
uint32_t telemetria_descartes_usb() { return descartesUsb; }

// Entrada de comandos.
// Os comandos do painel entram em um buffer circular (anel), e o debug.cpp lê tudo pela
// entrada_le() como se fosse a serial. A USB tem prioridade sobre o anel.
static char     anel[256];
static uint16_t anelIni = 0, anelFim = 0;
static void __attribute__((unused)) anelPoe(char c) {
  uint16_t prox = (anelFim + 1) % sizeof(anel);
  if (prox != anelIni) { anel[anelFim] = c; anelFim = prox; }   // com o anel cheio o caractere é perdido
}
int entrada_le() {
  if (Serial.available()) return Serial.read();
  if (anelIni != anelFim) { char c = anel[anelIni]; anelIni = (anelIni + 1) % sizeof(anel); return (uint8_t)c; }
  return -1;
}

#if USAR_WIFI
#include <WiFi.h>
#include <ESPmDNS.h>
#include <lwip/sockets.h>
#include <errno.h>
#include "src/painel_gz.h"         // painel HTML comprimido em gzip (gerado pelo doc/gera_painel.py)

#define MAX_PAINEIS 2
static WiFiServer srv(80);
static WiFiClient cli;               // requisição que está sendo lida
static WiFiClient painel[MAX_PAINEIS];   // conexões abertas do /stream
static char       req[200];          // primeira linha da requisição HTTP
static uint8_t    nReq = 0;
static bool       linha1Ok = false, linhaVazia = false;
static uint32_t   tCli = 0;          // momento em que a requisição atual chegou
static bool       novoPainel = false;
static uint32_t   descartes = 0;     // linhas descartadas no WiFi por causa da rede cheia

// Saída para os painéis.
// A linha ganha o formato do SSE ("data: ...\n\n") e vai para cada painel sem esperar.
static void enviaPaineis(const char* s, uint16_t n) {
  bool temPainel = false;
  for (uint8_t k = 0; k < MAX_PAINEIS; k++) if (painel[k]) temPainel = true;
  if (!temPainel) return;
  static char msg[sizeof(lbuf) + 10];
  if (n > sizeof(msg) - 10) n = sizeof(msg) - 10;
  memcpy(msg, "data: ", 6);
  memcpy(msg + 6, s, n);
  msg[6 + n] = '\n'; msg[7 + n] = '\n';
  int tot = 8 + n;
  for (uint8_t k = 0; k < MAX_PAINEIS; k++) {
    if (!painel[k]) continue;
    int r = send(painel[k].fd(), msg, tot, MSG_DONTWAIT);   // send do lwip direto no socket, sem bloquear
    if (r == tot) continue;
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { descartes++; continue; }   // rede cheia: linha descartada
    if (r < 0) painel[k].stop();                                                          // painel desconectado
  }
}


// Servidor HTTP mínimo, só com o necessário para o painel.
static void responde(WiFiClient& c, const char* status, const char* tipo, const char* corpo) {
  c.printf("HTTP/1.1 %s\r\nContent-Type: %s\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-cache\r\n"
           "Connection: close\r\nContent-Length: %u\r\n\r\n%s", status, tipo, (unsigned)strlen(corpo), corpo);
}

// conversão de um dígito hexadecimal em número, usada para decodificar o %XX da URL
static int hex(char c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : 0; }

// Tratamento de uma requisição completa. A primeira linha tem o formato "GET /caminho?consulta HTTP/1.1".
static void atende() {
  if (strncmp(req, "GET ", 4) != 0) { responde(cli, "405 Method Not Allowed", "text/plain", "so GET"); cli.stop(); return; }
  char* caminho = req + 4;
  char* fim = strchr(caminho, ' ');
  if (fim) *fim = 0;
  char* consulta = strchr(caminho, '?');
  if (consulta) *consulta++ = 0;               // separação entre o caminho e a consulta

  // "/" entrega o painel. O envio pode demorar, então só acontece com o robô parado (ESPERA).
  if (!strcmp(caminho, "/") || !strcmp(caminho, "/index.html")) {
    if (estado_atual() != ESPERA) { responde(cli, "503 Service Unavailable", "text/plain", "Pare o robo (stop) para abrir o painel."); cli.stop(); return; }
    cli.setNoDelay(false);
    cli.printf("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Encoding: gzip\r\n"
               "Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)PAINEL_GZ_LEN);
    size_t env = 0;
    uint32_t t0 = millis();
    // o laço manda o arquivo em pedaços do tamanho de um pacote TCP, com limite de 8 s
    while (env < PAINEL_GZ_LEN && cli.connected() && millis() - t0 < 8000) {      // robô parado: o bloqueio aqui não atrapalha
      size_t n = min((size_t)1436, PAINEL_GZ_LEN - env);
      size_t w = cli.write(PAINEL_GZ + env, n);
      if (w == 0) delay(2); else env += w;
    }
    cli.stop();
    return;
  }
  // "/stream" mantém a conexão aberta e passa a receber todas as linhas do Log
  if (!strcmp(caminho, "/stream")) {
    cli.setNoDelay(true);
    cli.print("HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-cache\r\n"
              "Connection: keep-alive\r\nAccess-Control-Allow-Origin: *\r\n\r\nretry: 1000\n\n");
    uint8_t k = 0;
    while (k < MAX_PAINEIS && painel[k]) k++;                      // busca de uma vaga livre
    if (k == MAX_PAINEIS) { painel[0].stop(); k = 0; }             // sem vaga: o painel mais antigo sai
    painel[k] = cli;
    cli = WiFiClient();
    novoPainel = true;
    return;
  }
  // "/cmd?c=..." decodifica o comando e coloca no anel, como se tivesse sido digitado
  if (!strcmp(caminho, "/cmd")) {
    // c vazio equivale a apertar ENTER
    const char* p = consulta ? strstr(consulta, "c=") : nullptr;
    if (p) {
      p += 2;
      while (*p && *p != '&') {
        char ch = *p++;
        if (ch == '+') ch = ' ';
        else if (ch == '%' && p[0] && p[1]) { ch = (char)(hex(p[0]) * 16 + hex(p[1])); p += 2; }
        if (ch != '\n' && ch != '\r') anelPoe(ch);
      }
    }
    anelPoe('\n');
    responde(cli, "200 OK", "text/plain", "ok");
    cli.stop();
    return;
  }
  if (!strcmp(caminho, "/ping")) { responde(cli, "200 OK", "text/plain", "pong"); cli.stop(); return; }
  responde(cli, "404 Not Found", "text/plain", "nao existe");
  cli.stop();
}

// Inicialização do WiFi. Com WIFI_STA_SSID preenchido, o ESP32 tenta o roteador por até 8 s;
// sem conexão, ele cria a rede própria (modo AP).
void telemetria_init() {
  WiFi.persistent(false);                     // a configuração do WiFi não vai para a flash
  bool sta = false;
  if (strlen(WIFI_STA_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_SENHA);
    Log.printf("# WiFi: entrando na rede \"%s\"", WIFI_STA_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) { delay(250); Log.print("."); }
    Log.println();
    sta = WiFi.status() == WL_CONNECTED;
    if (!sta) Log.println(F("! nao conectou no roteador: criando a rede propria"));
  }
  if (!sta) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_SENHA, WIFI_AP_CANAL, 0, MAX_PAINEIS + 1);
  }
  WiFi.setSleep(false);                       // sem economia de energia = menor atraso
  if (MDNS.begin(WIFI_NOME_MDNS)) MDNS.addService("http", "tcp", 80);
  srv.begin();
  srv.setNoDelay(true);
  IPAddress ip = sta ? WiFi.localIP() : WiFi.softAPIP();
  if (sta) Log.printf("# WiFi OK (roteador). Painel: http://%s  ou  http://%s.local\n", ip.toString().c_str(), WIFI_NOME_MDNS);
  else     Log.printf("# WiFi OK: conecte na rede \"%s\" (senha %s) e abra http://%s\n", WIFI_AP_SSID, WIFI_AP_SENHA, ip.toString().c_str());
}

// O loop do sumo_esp32.ino chama esta função em toda volta. O servidor aceita no máximo uma
// requisição por vez e lê só o que já chegou, guardando a primeira linha e pulando os cabeçalhos.
void telemetria_passo() {
  if (!cli) {
    cli = srv.accept();
    if (cli) { nReq = 0; linha1Ok = false; linhaVazia = false; tCli = millis(); }
  }
  if (cli) {
    while (cli.available()) {
      char ch = cli.read();
      if (!linha1Ok) {
        if (ch == '\n') { req[nReq] = 0; linha1Ok = true; linhaVazia = true; }
        else if (ch != '\r' && nReq < sizeof(req) - 1) req[nReq++] = ch;
      } else {                                   // cabeçalhos ignorados até a linha vazia
        if (ch == '\n') { if (linhaVazia) { atende(); break; } linhaVazia = true; }
        else if (ch != '\r') linhaVazia = false;
      }
    }
    if (cli && millis() - tCli > 1500) cli.stop();   // requisição incompleta: conexão encerrada
  }
}

bool telemetria_novo_painel() { bool r = novoPainel; novoPainel = false; return r; }
uint8_t telemetria_paineis() { uint8_t n = 0; for (uint8_t k = 0; k < MAX_PAINEIS; k++) if (painel[k]) n++; return n; }

#else
// Versão sem WiFi: as mesmas funções existem, mas não fazem nada.
static void enviaPaineis(const char*, uint16_t) {}
void telemetria_init() {}
void telemetria_passo() {}
bool telemetria_novo_painel() { return false; }
uint8_t telemetria_paineis() { return 0; }
#endif
