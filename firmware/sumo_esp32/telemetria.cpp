// telemetria.cpp
// Saída de texto e entrada de comandos pela USB e, com USAR_WIFI 1, também pelo WiFi.
//
// O WiFi usa só as bibliotecas que já vêm no core do ESP32. A escolha foi Server-Sent Events em
// vez de WebSocket porque o navegador reconecta sozinho, nenhuma biblioteca extra precisa ser
// instalada na IDE e o código funciona igual nos cores 2.x e 3.x. Os comandos do painel chegam
// por /cmd, em uma requisição curta.
//
// Com USAR_WIFI 0 tudo que é do rádio fica fora da compilação (#if USAR_WIFI): o binário é o mesmo
// da versão só com USB. Para manter o tempo real sem travar o robô, quatro cuidados foram tomados:
//   WiFi.setSleep(false) deixa o rádio sempre acordado (dormir soma uns 100 ms de atraso);
//   setNoDelay faz cada linha sair na hora, sem esperar para juntar pacotes;
//   o envio usa MSG_DONTWAIT, então com a rede cheia a linha de dados é descartada em vez de
//   segurar o loop (a telemetria perde um quadro, mas o robô nunca espera);
//   o aceite de conexões e a leitura de requisições rodam no máximo a cada WIFI_PASSO_MS.

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
static void enviaPaineis(const char* s, uint16_t n, bool dado);

// Envio da linha acumulada. Linhas de texto comum sempre saem na USB; linhas de dados só saem
// quando cabem no buffer da serial. Para o painel pelo WiFi a regra é a da rede (enviaPaineis).
static void fechaLinha() {
  bool dado = lN >= 2 && lbuf[1] == ',' && (lbuf[0] == 'D' || lbuf[0] == 'Q' || lbuf[0] == 'M' || lbuf[0] == 'J' || lbuf[0] == 'Z');
  if (!dado || Serial.availableForWrite() > (int)lN + 2) {
    Serial.write((const uint8_t*)lbuf, lN);
    Serial.write('\n');
  } else descartesUsb++;
  enviaPaineis(lbuf, lN, dado);
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
// Os comandos do painel pelo WiFi entram em um buffer circular (anel), e o debug.cpp lê tudo pela
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

#define MAX_PAINEIS   2
#define WIFI_PASSO_MS 3                  // intervalo mínimo entre duas atendidas da rede

// Um painel conectado no /stream. Se a rede aceita só parte de uma linha, o resto fica em "resto" e
// sai primeiro na próxima chamada; sem isso o quadro seguinte começaria no meio de outro.
struct Painel {
  WiFiClient c;
  uint16_t   nResto;
  char       resto[sizeof(lbuf) + 16];
};
static WiFiServer srv(80);
static WiFiClient cli;                   // requisição que está sendo lida
static Painel     painel[MAX_PAINEIS];
static char       req[200];              // primeira linha da requisição HTTP
static uint8_t    nReq = 0;
static bool       linha1Ok = false, linhaVazia = false;
static uint32_t   tCli = 0;              // momento em que a requisição atual chegou
static uint32_t   descartes = 0;         // linhas de dados descartadas no WiFi por causa da rede cheia
static uint32_t   tPasso = 0;
static bool       ligado = false;        // rádio ligado
static bool       modoSta = false;       // true quando entrou no roteador, false na rede própria

uint32_t telemetria_descartes_wifi() { return descartes; }
bool     telemetria_wifi_ligado() { return ligado; }
uint8_t  telemetria_paineis() {
  uint8_t n = 0;
  for (uint8_t k = 0; k < MAX_PAINEIS; k++) if (painel[k].c) n++;
  return n;
}

// Tenta mandar "n" bytes sem esperar. Devolve quantos saíram (pode ser 0) ou -1 se o painel caiu.
static int manda(Painel& p, const char* b, int n) {
  int r = send(p.c.fd(), b, n, MSG_DONTWAIT);        // send do lwip direto no socket, sem bloquear
  if (r >= 0) return r;
  if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
  return -1;
}

// Saída para os painéis.
// A linha ganha o formato do SSE ("data: ...\n\n") e vai para cada painel sem esperar. Linha de
// dados que não cabe agora é descartada; linha de texto (resultado de comando, aviso) que só coube
// em parte guarda o resto para a próxima volta, porque perder o meio de uma linha embaralharia o painel.
static void enviaPaineis(const char* s, uint16_t n, bool dado) {
  if (!ligado) return;
  bool temPainel = false;
  for (uint8_t k = 0; k < MAX_PAINEIS; k++) if (painel[k].c) temPainel = true;
  if (!temPainel) return;
  static char msg[sizeof(lbuf) + 10];
  if (n > sizeof(lbuf)) n = sizeof(lbuf);
  memcpy(msg, "data: ", 6);
  memcpy(msg + 6, s, n);
  msg[6 + n] = '\n'; msg[7 + n] = '\n';
  int tot = 8 + n;
  for (uint8_t k = 0; k < MAX_PAINEIS; k++) {
    Painel& p = painel[k];
    if (!p.c) continue;
    if (p.nResto) { descartes++; continue; }         // ainda devendo parte da linha anterior: esta fica de fora
    int r = manda(p, msg, tot);
    if (r < 0) { p.c.stop(); continue; }             // painel desconectado
    if (r == tot) continue;
    if (dado && r == 0) { descartes++; continue; }   // rede cheia e linha de dados: descartada inteira
    memcpy(p.resto, msg + r, tot - r);               // coube só parte (ou é texto importante): guarda o resto
    p.nResto = tot - r;
  }
}

// Envio do que ficou pendente de uma linha anterior.
static void descarrega() {
  for (uint8_t k = 0; k < MAX_PAINEIS; k++) {
    Painel& p = painel[k];
    if (!p.c || !p.nResto) continue;
    int r = manda(p, p.resto, p.nResto);
    if (r < 0) { p.c.stop(); p.nResto = 0; continue; }
    if (r > 0) { memmove(p.resto, p.resto + r, p.nResto - r); p.nResto -= r; }
  }
}

// Servidor HTTP mínimo, só com o necessário para o painel. O navegador abre o painel_sumo.html
// do disco (origem diferente do robô), então toda resposta libera o CORS.
static void responde(WiFiClient& c, const char* status, const char* tipo, const char* corpo) {
  c.printf("HTTP/1.1 %s\r\nContent-Type: %s\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Private-Network: true\r\n"
           "Cache-Control: no-cache\r\nConnection: close\r\nContent-Length: %u\r\n\r\n%s",
           status, tipo, (unsigned)strlen(corpo), corpo);
}

// conversão de um dígito hexadecimal em número, usada para decodificar o %XX da URL
static int hex(char c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : 0; }

// Tratamento de uma requisição completa. A primeira linha tem o formato "GET /caminho?consulta HTTP/1.1".
static void atende() {
  // OPTIONS é a consulta prévia (preflight) que alguns navegadores fazem antes de falar com um endereço da rede local
  if (strncmp(req, "OPTIONS ", 8) == 0) {
    cli.print("HTTP/1.1 204 No Content\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Private-Network: true\r\n"
              "Access-Control-Allow-Methods: GET, OPTIONS\r\nAccess-Control-Allow-Headers: *\r\nConnection: close\r\n\r\n");
    cli.stop();
    return;
  }
  if (strncmp(req, "GET ", 4) != 0) { responde(cli, "405 Method Not Allowed", "text/plain", "so GET"); cli.stop(); return; }
  char* caminho = req + 4;
  char* fim = strchr(caminho, ' ');
  if (fim) *fim = 0;
  char* consulta = strchr(caminho, '?');
  if (consulta) *consulta++ = 0;               // separação entre o caminho e a consulta

  // "/stream" mantém a conexão aberta e passa a receber todas as linhas do Log
  if (!strcmp(caminho, "/stream")) {
    cli.setNoDelay(true);
    cli.print("HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-cache\r\n"
              "Connection: keep-alive\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Private-Network: true\r\n\r\nretry: 1000\n\n");
    uint8_t k = 0;
    while (k < MAX_PAINEIS && painel[k].c) k++;                    // busca de uma vaga livre
    if (k == MAX_PAINEIS) { painel[0].c.stop(); k = 0; }           // sem vaga: o painel mais antigo sai
    painel[k].c = cli;
    painel[k].nResto = 0;
    cli = WiFiClient();
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
  ligado = true; modoSta = sta;
  IPAddress ip = sta ? WiFi.localIP() : WiFi.softAPIP();
  if (sta) Log.printf("# WiFi OK (roteador). Painel: IP %s ou %s.local\n", ip.toString().c_str(), WIFI_NOME_MDNS);
  else     Log.printf("# WiFi OK: conecte na rede \"%s\" (senha %s) e use o endereco %s no painel\n", WIFI_AP_SSID, WIFI_AP_SENHA, ip.toString().c_str());
}

void telemetria_wifi_desliga(const char* motivo) {
  if (!ligado) return;
  Log.printf("# WiFi desligado: %s\n", motivo);   // a linha sai antes de cortar a conexão dos painéis
  ligado = false;
  for (uint8_t k = 0; k < MAX_PAINEIS; k++) { painel[k].c.stop(); painel[k].nResto = 0; }
  cli.stop();
  srv.end();
  MDNS.end();
  if (!modoSta) WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

void telemetria_wifi_info() {
  if (!ligado) { Log.println(F("# WiFi desligado (USAR_WIFI 1, mas o radio foi desligado; reinicie o ESP32 para ligar de novo)")); return; }
  IPAddress ip = modoSta ? WiFi.localIP() : WiFi.softAPIP();
  Log.printf("# WiFi: %s | rede \"%s\" | IP %s | %u painel(is) conectado(s) | stream a %d ms | linhas descartadas %lu\n",
             modoSta ? "roteador" : "rede propria (AP)", modoSta ? WIFI_STA_SSID : WIFI_AP_SSID, ip.toString().c_str(),
             telemetria_paineis(), STREAM_MS, (unsigned long)descartes);
}

// O loop do sumo_esp32.ino chama esta função em toda volta. O servidor aceita no máximo uma
// requisição por vez e lê só o que já chegou, guardando a primeira linha e pulando os cabeçalhos.
void telemetria_passo() {
  if (!ligado) return;
#if WIFI_DESLIGA_SEM_PAINEL
  // a luta começou (saiu da ESPERA) e ninguém está olhando: o rádio não precisa continuar. O ajuste "wifi" do painel
  // (ajuste wifi 1) mantém o rádio ligado na luta, para acompanhar pelo painel mesmo conectando depois do start.
  if (estado_atual() != ESPERA && telemetria_paineis() == 0 && !ajustes().wifi_luta) { telemetria_wifi_desliga("luta iniciada sem painel conectado"); return; }
#endif
  uint32_t agora = millis();
  descarrega();
  if (agora - tPasso < WIFI_PASSO_MS) return;
  tPasso = agora;
  if (!cli) {
    cli = srv.accept();
    if (cli) { nReq = 0; linha1Ok = false; linhaVazia = false; tCli = agora; }
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
    if (cli && agora - tCli > 1500) cli.stop();   // requisição incompleta: conexão encerrada
  }
}

#else
// Versão sem WiFi: as mesmas funções existem, mas não fazem nada e nada do rádio entra no binário.
static void enviaPaineis(const char*, uint16_t, bool) {}
void     telemetria_init() {}
void     telemetria_passo() {}
uint8_t  telemetria_paineis() { return 0; }
bool     telemetria_wifi_ligado() { return false; }
void     telemetria_wifi_desliga(const char*) {}
void     telemetria_wifi_info() { Log.println(F("# WiFi desligado no firmware (USAR_WIFI 0 em config.h)")); }
uint32_t telemetria_descartes_wifi() { return 0; }
#endif
