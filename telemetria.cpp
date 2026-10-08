// telemetria.cpp
// Saída de texto e entrada de comandos, só pela USB.
// O robô compete de forma autônoma, então o firmware não liga o rádio: sem a pilha do WiFi,
// o núcleo 0 fica livre para o rastreador e sobra mais RAM e flash para o resto do código.

#include "telemetria.h"
#include "config.h"

Saida Log;

// Saída por linha inteira.
// O texto fica acumulado até o '\n' e só então sai inteiro pela USB. As linhas de dados
// (D, Q e M) são descartadas quando o buffer de envio está cheio, já que a 115200 baud cabem
// só uns 11 KB/s; assim o robô nunca fica esperando a serial. O buffer de 4 KB da serial é
// criado no sumo_esp32.ino, antes do Serial.begin.
static char     lbuf[1024];          // tamanho suficiente para a linha do mapa (até uns 800 caracteres)
static uint16_t lN = 0;              // quantidade de caracteres na linha atual
static uint32_t descartesUsb = 0;

// Envio da linha acumulada. Linhas de texto comum sempre saem; linhas de dados só saem
// quando cabem no buffer da serial.
static void fechaLinha() {
  bool dado = lN >= 2 && lbuf[1] == ',' && (lbuf[0] == 'D' || lbuf[0] == 'Q' || lbuf[0] == 'M');
  if (!dado || Serial.availableForWrite() > (int)lN + 2) {
    Serial.write((const uint8_t*)lbuf, lN);
    Serial.write('\n');
  } else descartesUsb++;
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

// Entrada de comandos: o Monitor Serial e o painel escrevem na mesma porta USB.
int entrada_le() {
  if (Serial.available()) return Serial.read();
  return -1;
}
