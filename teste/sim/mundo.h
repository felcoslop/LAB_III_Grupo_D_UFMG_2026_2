// mundo.h
// Mundo do simulador de luta: o dojô da CoRA (77 cm, faixa branca de 2,5 cm), os dois robôs
// como corpos rígidos em 2D e os sensores do nosso robô. O firmware de verdade roda em cima
// deste mundo pelo hal.cpp, que entrega o relógio, a borda, os ToF e recebe o PWM dos motores.
//
// Física (passo de 0,25 ms): cada roda tem motor DC com força contraeletromotriz, redução e
// atrito com o chão limitado por mu.N (a roda patina quando o motor pede mais que isso); o
// contato entre os robôs é uma mola amortecida na direção de menor penetração (teorema do eixo
// separador), com atrito; a lâmina mais baixa tira parte do peso das rodas do outro robô.
// Unidades internas no SI (m, s, kg, N); os sensores trabalham em mm, como o firmware.
#pragma once
#include <stdint.h>
#include <vector>
#include <string>
#include <random>

namespace sim {

const double R_DOJO  = 0.385;    // raio do dojô (m)
const double FAIXA   = 0.025;    // largura da faixa branca (m)
const double G       = 9.81;

// Parâmetros de um robô (o nosso e cada tipo de oponente)
struct Param {
  std::string nome;
  double massa = 1.0;            // kg
  double meioX = 0.076, meioY = 0.076;   // metade do comprimento e da largura (m)
  double bitola = 0.132, raio = 0.016;   // rodas (m)
  double fracRodas = 0.8;        // parte do peso nas rodas de tração (o resto na lâmina e no apoio)
  double mu = 1.0;               // atrito do pneu com o dojô
  double muApoio = 0.25;         // atrito da lâmina e do apoio arrastando no dojô
  double lamina = 0.5;           // qualidade da lâmina (0 alta, 1 rente ao chão)
  // motor + redução, vistos no eixo da roda
  double vNom = 12.0, rpmLivre = 319, iLivre = 0.1, iTravado = 1.2, torqueTravado = 0.186;
  double jRoda = 4e-5;           // inércia da roda e do rotor refletida no eixo (kg.m²)
  double vBat = 11.1, quedaPonte = 0.2;
  double refl = 0.6;             // refletância para o ToF (0,6 claro, 0,05 preto fosco)
  bool   furtivo = false;        // laterais inclinadas que desviam o infravermelho
  bool   freado = false;         // motores desligados com redução que não gira de fora para dentro
};

Param paramNosso();
Param paramOponente(const std::string& tipo);   // parado, pesado, rapido, lento, errante, espelho

struct Roda { double w = 0; double v = 0; };    // velocidade angular (rad/s) e tensão aplicada (V)

struct Robo {
  Param  p;
  double x = 0, y = 0, th = 0;   // posição (m) e rumo (rad) no referencial do dojô
  double vx = 0, vy = 0, w = 0;  // velocidades (m/s e rad/s)
  Roda   rodaE, rodaD;
  double cargaExtra = 0;         // carga a mais (+) ou a menos (-) nas rodas, em N, pelo contato
  bool   fora = false;
  double tFora = -1;
  void   cantos(double cx[4], double cy[4]) const;
  void   paraMundo(double bx, double by, double& wx, double& wy) const;   // ponto do corpo (m) para o mundo
  void   paraCorpo(double wx, double wy, double& bx, double& by) const;
};

// Objeto parado fora do dojô (pessoa, juiz, parede): cilindro visto pelos ToF
struct Objeto { double x, y, raio, refl; };

// Modelo do ToF: alcance em alvo escuro, ruído e leituras falsas mudam com o perfil
struct PerfilToF { double limiar; double ruido; double falsa; double alcanceMax; };

class Mundo {
public:
  Mundo(uint32_t semente);
  Robo nosso, op;
  std::vector<Objeto> fora;      // objetos fora do dojô
  double t = 0;                  // tempo da simulação (s)
  bool   contato = false;        // os dois robôs encostados agora
  double tUltContato = -10;
  double tContatoTotal = 0;
  std::mt19937 rng;
  double arranhoes = 0.0005;     // chance por ms de um arranhão passar por um sensor de borda

  void passo(double dt);         // avanço da física
  // sensores do nosso robô
  double bordaMv(double bx, double by);                   // TCRT5000 no ponto do corpo (mm)
  int    tofMede(double sx, double sy, double angGraus, const PerfilToF& pf, int* quem);   // mm, 8190 sem nada
  double alcanceVisivel(double sx, double sy, double angGraus, int* quem);   // distância geométrica sem ruído
  double uniforme(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
  double normal(double s) { return std::normal_distribution<double>(0.0, s)(rng); }

private:
  void   forcasRobo(Robo& r, double& fx, double& fy, double& tq, double dt);
  void   contatoRobos(double& fxA, double& fyA, double& tqA, double& fxB, double& fyB, double& tqB);
  void   integra(Robo& r, double fx, double fy, double tq, double dt);
  void   confereFora(Robo& r);
};

}  // namespace sim
