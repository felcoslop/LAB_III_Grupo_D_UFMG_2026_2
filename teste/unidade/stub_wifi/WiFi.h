// WiFi.h de mentira para compilar e testar o telemetria.cpp (USAR_WIFI 1) no computador. As classes
// usam sockets POSIX de verdade (não bloqueantes), então o servidor do firmware conversa com um
// cliente TCP real dentro do próprio teste. A porta pedida pelo firmware (80) vira 8080, para não
// precisar de permissão de administrador.
#pragma once
#include <Arduino.h>
#include <string>
#include <memory>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>

inline void delay(unsigned long) {}

struct IPAddress {
  std::string txt;
  IPAddress(const char* t = "0.0.0.0") : txt(t) {}
  std::string toString() const { return txt; }
};

class WiFiClient : public Print {
  struct Sock { int fd; explicit Sock(int f) : fd(f) {} ~Sock() { if (fd >= 0) ::close(fd); } };
  std::shared_ptr<Sock> s;
public:
  WiFiClient() {}
  explicit WiFiClient(int fd) : s(std::make_shared<Sock>(fd)) {}
  int fd() const { return s ? s->fd : -1; }
  bool connected() {
    if (fd() < 0) return false;
    char c;
    ssize_t r = ::recv(fd(), &c, 1, MSG_PEEK | MSG_DONTWAIT);
    if (r == 0) return false;                         // o outro lado fechou
    return r > 0 || errno == EAGAIN || errno == EWOULDBLOCK;
  }
  operator bool() { return connected(); }
  int available() { int n = 0; if (fd() >= 0) ioctl(fd(), FIONREAD, &n); return n; }
  int read() { unsigned char c; return fd() >= 0 && ::recv(fd(), &c, 1, MSG_DONTWAIT) == 1 ? c : -1; }
  void setNoDelay(bool on) { int v = on; if (fd() >= 0) setsockopt(fd(), IPPROTO_TCP, TCP_NODELAY, &v, sizeof(v)); }
  void stop() { if (s && s->fd >= 0) { ::close(s->fd); s->fd = -1; } }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* b, size_t n) override {
    if (fd() < 0) return 0;
    ssize_t r = ::send(fd(), b, n, MSG_NOSIGNAL);
    return r > 0 ? (size_t)r : 0;
  }
  size_t printf(const char* f, ...) {                  // buffer maior que o do Print de mentira
    char buf[1024]; va_list a; va_start(a, f); int n = vsnprintf(buf, sizeof(buf), f, a); va_end(a);
    return n > 0 ? write((const uint8_t*)buf, strlen(buf)) : 0;
  }
  using Print::print;
};

class WiFiServer {
  int lfd = -1; uint16_t porta;
public:
  explicit WiFiServer(uint16_t p) : porta(p) {}
  void begin() {
    signal(SIGPIPE, SIG_IGN);
    lfd = ::socket(AF_INET, SOCK_STREAM, 0);
    int um = 1; setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &um, sizeof(um));
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_LOOPBACK); a.sin_port = htons(porta + 8000);
    if (::bind(lfd, (sockaddr*)&a, sizeof(a)) != 0 || ::listen(lfd, 4) != 0) { perror("bind/listen"); ::close(lfd); lfd = -1; return; }
    fcntl(lfd, F_SETFL, O_NONBLOCK);
  }
  void setNoDelay(bool) {}
  WiFiClient accept() {
    if (lfd < 0) return WiFiClient();
    int f = ::accept(lfd, nullptr, nullptr);
    if (f < 0) return WiFiClient();
    int pequeno = 4096; setsockopt(f, SOL_SOCKET, SO_SNDBUF, &pequeno, sizeof(pequeno));   // enche rápido, como a rede do ESP32
    return WiFiClient(f);
  }
  void end() { if (lfd >= 0) { ::close(lfd); lfd = -1; } }
};

enum { WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2 };
enum { WL_CONNECTED = 3, WL_DISCONNECTED = 6 };

struct WiFiFalso {
  void persistent(bool) {}
  bool mode(int) { return true; }
  bool softAP(const char*, const char*, int, int, int) { return true; }
  void begin(const char*, const char*) {}
  int  status() { return WL_DISCONNECTED; }
  bool setSleep(bool) { return true; }
  IPAddress softAPIP() { return IPAddress("192.168.4.1"); }
  IPAddress localIP() { return IPAddress("192.168.0.50"); }
  bool softAPdisconnect(bool) { return true; }
  bool disconnect(bool) { return true; }
};
extern WiFiFalso WiFi;
