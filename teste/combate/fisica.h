// fisica.h
// Mundo do simulador de combate: dojô da RoboCore para as classes Lego/Vex de 1 kg (77 cm de
// diâmetro, 2,5 cm de altura, faixa branca de 2,5 cm, madeira laminada de alta pressão), dois
// robôs com a geometria do STL do projeto mecânico e a sala onde a luta acontece.
//
// Cada robô apoia em três pontos: os dois pneus e a ponta da rampa, que arrasta no dojô. A carga
// em cada apoio sai do equilíbrio de momentos com o peso no centro de massa, a aceleração do
// passo anterior, a força do contato e a força vertical que a rampa do outro robô faz quando
// entra por baixo. A força de cada pneu sai do escorregamento do pneu (atrito com a curva de
// Stribeck, limitado por mu.N, nas direções longitudinal e lateral juntas), e cada roda tem o
// modelo do motor DC com força contraeletromotriz, redução e queda da bateria.
// Unidades internas no SI (m, s, kg, N); os sensores trabalham em mm, como o firmware.
#pragma once
#include <stdint.h>
#include <random>
#include <vector>

namespace cb {

const double R_DOJO = 0.385;     // raio do dojô (m)
const double FAIXA  = 0.025;     // largura da faixa branca (m)
const double G      = 9.81;

// Geometria do robô no referencial do corpo: origem no meio do eixo das rodas, no chão, x para
// a frente e y para a esquerda (medidas do STL e do desenho de conjunto do projeto mecânico).
const double X_PONTA   = 0.1177;   // ponta da rampa
const double X_FACE    = 0.100;    // face inclinada (61,8 graus) na altura das lentes dos ToF
const double X_TRAS    = -0.025;   // face de trás da base
const double Y_LADO    = 0.0745;   // metade da largura do corpo
const double BITOLA    = 0.1273;   // centros dos pneus depois do corte de 2 mm na ponta dos eixos
const double R_RODA    = 0.01597;  // pneu StickyMAX S20 de 32 mm
const double Y_PNEU_IN = 0.0535;   // face de dentro do pneu
const double X_PNEU    = 0.016;    // metade do comprimento do pneu visto de cima
const double ANG_RAMPA = 17.7;     // graus

// Motor e redução vistos no eixo da roda (tabela do fabricante)
struct Motor {
  const char* nome;
  double vNom, rpmLivre, iLivre, iTravado, torqueTravado;   // V, rpm, A, A, N.m
  double jRoda;                                             // inércia no eixo da roda (kg.m²)
};
Motor motor319();     // JGB37-520 12 V, 319 rpm, redução 18,8:1, linha de 6000 rpm
Motor motor1000();    // JGB37-520 12 V, 1000 rpm, redução 10:1, linha de 10000 rpm

struct Param {
  double massa  = 0.98;
  double xCg    = 0.0269;   // centro de massa à frente do eixo (balanço de massa do projeto)
  double hCg    = 0.024;    // altura do centro de massa
  double izz    = 2.5e-3;   // inércia de giro no centro de massa (kg.m²)
  Motor  motor;
  double muEst  = 1.10;     // pneu de elastômero macio na madeira laminada, parado
  double muDin  = 0.95;     // e escorregando
  double vStrib = 0.03;     // m/s, transição entre os dois
  double cRol   = 0.015;    // resistência ao rolamento do pneu macio
  double muPla  = 0.35;     // ponta da rampa (PLA) arrastando no dojô
  double muPP   = 0.30;     // PLA contra PLA no contato entre os robôs
  double refl   = 0.05;     // refletância do corpo no infravermelho (PLA preto 0,05, branco 0,6)
  double vAberta = 12.3;    // bateria 3S sem carga (V)
  double rBat   = 0.15;     // resistência da bateria, do BMS e dos fios (ohm)
  double queda  = 0.2;      // queda na ponte H de MOSFET (V)
  double ponta  = 0.3e-3;   // altura da ponta da rampa sobre o dojô (m), varia na impressão
  // diferença de fábrica entre os dois motorredutores (fração da rotação livre e do torque)
  double varRpm[2] = {0, 0}, varTorque[2] = {0, 0};
};

struct Lift { double v = 0, x = 0, y = 0; };   // força vertical (N, + para cima) num ponto do corpo

struct Robo {
  Param  p;
  double x = 0, y = 0, th = 0;     // posição do centro de massa (m) e rumo (rad)
  double vx = 0, vy = 0, w = 0;
  double wE = 0, wD = 0;           // rodas (rad/s)
  double angE = 0, angD = 0;       // ângulo acumulado das rodas, para o 3D
  double dutyE = 0, dutyD = 0;     // PWM de -1 a 1
  double iE = 0, iD = 0;           // corrente nos motores (A)
  double vBat = 12.3;              // bateria com carga
  double nE = 0, nD = 0, nS = 0;   // carga nos apoios (N)
  double nS0 = 0;                  // carga na ponta parada, para comparar
  double ax = 0, ay = 0;           // aceleração no referencial do corpo (passo anterior)
  double fcx = 0, fcy = 0;         // força do contato no referencial do corpo
  double xApoio = X_PONTA;         // apoio da frente (vai para a quina do dojô se a ponta passar)
  Lift   lift;                     // rampa do outro robô levantando este
  Lift   prensa;                   // este robô empurrando o outro para cima (reação para baixo)
  double fTracE = 0, fTracD = 0;   // força longitudinal de cada pneu (N)
  bool   fora = false;
  double tFora = -1;
  int    motivoFora = 0;           // 1 roda fora, 2 centro de massa fora, 3 tombou na quina
  double arfagem = 0, rolagem = 0; // ângulos só para o desenho 3D (rad)
  // ponto do corpo (origem no eixo) para o mundo e de volta
  void   eixo(double& ex, double& ey) const;
  void   paraMundo(double bx, double by, double& wx, double& wy) const;
  void   paraCorpo(double wx, double wy, double& bx, double& by) const;
};

// Pessoa parada fora do dojô (as duas pernas viram um cilindro para os ToF)
struct Pessoa { double x, y, raio, refl; };

struct PerfilToF { double limiar, ruido, falsa, alcanceMax; };

class Mundo {
public:
  explicit Mundo(uint32_t semente);
  Robo r[2];
  std::vector<Pessoa> pessoas;
  double salaX = 2.5, salaY = 2.1;  // paredes da sala (m a partir do centro do dojô)
  double reflParede = 0.6;
  double t = 0;
  bool   contato = false;
  double tUltContato = -10, tContatoTotal = 0;
  double fContato = 0;              // força normal no contato (N)
  double arranhoes = 0.0003;        // chance por ms de um risco ou poeira passar por um sensor de borda
  double ambienteToF = 0.85;        // sala à noite com lâmpada de LED: limiar de sinal menor
  std::mt19937 rng;

  void   passo(double dt);
  void   ajustaApoioInicial(Robo& rb);   // carga parada nos apoios (nS0)
  // sensores do robô k (posição no referencial do corpo em mm, como no config.h)
  double bordaMv(int k, double bxmm, double bymm);
  int    tofMede(int k, double sxmm, double symm, double angGraus, const PerfilToF& pf);
  double uniforme(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
  double normal(double s) { return std::normal_distribution<double>(0.0, s)(rng); }

private:
  void   contatoRobos(double f[2][3]);
  void   cargas(Robo& rb);
  void   forcasRobo(Robo& rb, double& fx, double& fy, double& tq, double dt);
  void   integra(Robo& rb, double fx, double fy, double tq, double dt);
  void   confereFora(Robo& rb);
  double liftMaximo(const Robo& rb, double bx, double by);
};

// estáticos usados pelo desenho e pelos testes
double kappaRampa(double muPP);       // força vertical por força horizontal na rampa
void   cantos(const Robo& rb, double cx[4], double cy[4]);

}  // namespace cb
