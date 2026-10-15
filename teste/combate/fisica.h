// fisica.h
// Mundo do simulador de combate: dojô de madeira pintada da classe Lego/Vex de 1 kg (77 cm de
// diâmetro, 2,5 cm de altura, faixa branca de 2,5 cm), dois robôs com a geometria do STL do
// projeto mecânico e a sala de Belo Horizonte onde a luta acontece (gravidade e ar do lugar).
//
// Cada robô apoia em três pontos: os dois pneus e a ponta da rampa, que arrasta no dojô. A carga
// em cada apoio sai do equilíbrio de momentos com o peso no centro de massa, a aceleração do
// passo anterior, a força do contato e a força vertical que a rampa do outro robô faz quando
// entra por baixo. A força de cada pneu sai do escorregamento do pneu (atrito com a curva de
// Stribeck, limitado por mu.N, nas direções longitudinal e lateral juntas), e cada roda tem o
// modelo do motor DC com força contraeletromotriz, redução e queda da bateria. O choque entre os
// dois corpos de PLA usa o modelo contínuo de contato com coeficiente de restituição de
// FLORES et al. (2011), e o ar freia cada robô com o arrasto de uma caixa.
// Unidades internas no SI (m, s, kg, N); os sensores trabalham em mm, como o firmware.
#pragma once
#include <stdint.h>
#include <random>
#include <vector>

namespace cb {

const double R_DOJO = 0.385;     // raio do dojô (m)
const double FAIXA  = 0.025;     // largura da faixa branca (m)
// Gravidade normal do GRS80 (MORITZ, 2000) na latitude da UFMG (-19,87 graus), menos a correção
// de ar livre de 0,3086 mGal/m e mais a da placa de Bouguer (2,67 g/cm³) para 830 m de altitude:
// 9,7863 - 0,0026 + 0,0009 = 9,7847 m/s² (HOFMANN-WELLENHOF; MORITZ, 2006).
const double G      = 9.7847;
// Ar da sala às 20 h de 2 de setembro: pressão da atmosfera padrão a 830 m (91,7 kPa, ISO 2533),
// 25 °C e 50% de umidade dão 1,065 kg/m³. Arrasto de caixa (Cd perto de 1, HOERNER, 1965) na
// área de frente do robô, 14,9 cm x 5,15 cm: a 1,5 m/s a força é de 0,009 N, menos de 0,1% do
// atrito do pneu, mas entra nas contas.
const double RHO_AR = 1.065;
const double CD_AR  = 1.0;
const double A_AR   = 0.149 * 0.0515;
// Geometria do robô no referencial do corpo: origem no meio do eixo das rodas, no chão, x para
// a frente e y para a esquerda (medidas do STL e do desenho de conjunto do projeto mecânico).
const double X_PONTA   = 0.1177;   // ponta da rampa
const double X_FACE    = 0.100;    // face inclinada (61,8 graus) na altura das lentes dos ToF
const double X_TRAS    = -0.025;   // face de trás da base
const double Y_LADO    = 0.0745;   // metade da largura do corpo
const double BITOLA    = 0.1273;   // centros dos pneus depois do corte de 2 mm na ponta dos eixos
const double R_RODA    = 0.01597;  // pneu StickyMAX S20 de 32 mm (desenho da RoboCore: 32 x 20 mm, 9,5 a 10 g)
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
Motor motor1000();    // JGB37-520 12 V, 1000 rpm, redução 6:1, a mesma linha de 6000 rpm

struct Param {
  double massa  = 0.98;
  double xCg    = 0.0269;   // centro de massa à frente do eixo (balanço de massa do projeto)
  double hCg    = 0.024;    // altura do centro de massa
  double izz    = 2.5e-3;   // inércia de giro no centro de massa (kg.m²)
  Motor  motor;
  // Pneu StickyMAX S20 (silicone de 20 Shore A, pegajoso) na madeira pintada. Borracha macia
  // numa superfície lisa tem atrito acima de 1, e o atrito escorregando é maior que o parado e
  // sobe com a velocidade de escorregamento: ROTH, DRISCOLL e HOLT (1942) mediram 1,2 a 2,0
  // parado em vidro e de 1,3 a 3,4 entre 1 µm/s e 5 cm/s escorregando; GROSCH (1963) mostrou
  // que o pico fica acima de 1 m/s para borracha de transição vítrea bem abaixo da temperatura
  // ambiente, como o silicone (-120 °C). A tinta é menos lisa e menos limpa que o vidro, e o pó
  // do dojô tira aderência do pneu pegajoso; por isso os valores ficam perto do mínimo dessas
  // medidas: 1,15 parado e 1,30 escorregando. A bateria roda também com o atrito 20% menor
  // (pó) e 20% maior. Medida no dojô de verdade: puxar o robô com as rodas travadas (ponte H em
  // freio) com um dinamômetro; mu = força / (massa x g).
  double muEst  = 1.15;     // parado
  double muDin  = 1.30;     // escorregando
  double vStrib = 0.05;     // m/s, transição entre os dois
  double cRol   = 0.015;    // resistência ao rolamento do pneu macio
  double muPla  = 0.35;     // ponta da rampa (PLA) arrastando no dojô
  double muPP   = 0.30;     // PLA contra PLA no contato entre os robôs
  // Choque entre os corpos de PLA. Os corpos impressos, com parafusos, placas e pneus, devolvem
  // pouco da energia; sem medida dos robôs, o coeficiente de restituição fica em 0,4 (a bateria
  // roda também com 0,2 e 0,6). Rigidez do contato medida no deslocamento do conjunto: 3e4 N/m.
  double restit = 0.4;
  double kContato = 3e4;
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
  double vImpacto = 0;              // velocidade de aproximação no começo do contato atual (m/s)
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
