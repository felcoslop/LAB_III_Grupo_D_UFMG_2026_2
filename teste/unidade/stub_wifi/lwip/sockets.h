// lwip/sockets.h de mentira (teste no computador): o send() é o do POSIX, mas o teste pode limitar
// quantos bytes cada chamada aceita (g_envio_max), para simular o envio parcial da rede cheia.
#pragma once
#include <sys/socket.h>
#include <errno.h>
extern int g_envio_max;                  // se maior que 0, cada send() aceita no máximo esse tanto de bytes
inline ssize_t send_teste(int fd, const void* b, size_t n, int fl) {
  size_t m = (g_envio_max > 0 && n > (size_t)g_envio_max) ? (size_t)g_envio_max : n;
  return ::send(fd, b, m, fl);
}
#define send(fd, b, n, fl) send_teste(fd, b, n, fl)
