# Montagem 3D: modelos, posições, fios e espaço no chassi

Esta pasta guarda os dados da página `site/montagem.html` (robô montado em 3D, com cada jumper),
que o `ferramentas/gera_montagem.py` gera. O `README.md` da raiz explica a página; aqui ficam o
fluxo de arquivos e a conferência de espaço no chassi de dois andares do editor 3D v11 (STL do
grupo, outubro de 2026).

## Arquivos

| Arquivo | Para que serve |
|---|---|
| `ferramentas/exporta_pecas.py` | lê o HTML do editor 3D e grava as malhas compactas (`pecas.js`, com a cor de cada vértice) e a matriz de cada peça repetida (`editor.json`, achada por ajuste de corpo rígido entre as cópias) |
| `montagem.json` | modelos (de quais peças do editor vem cada malha), peças, pinos de cada modelo, as duas emendas do barramento de I2C, passa-fios propostos, contorno do chassi e diferenças do esquema |
| `editor.json` | matrizes das peças repetidas (sete VL53L0X, quatro TCRT5000, células); gerado, não editar |
| `ferramentas/roteia_fios.py` | acha o caminho de cada fio por dentro do chassi (voxels de 1,5 mm, custo mínimo, folga de 1,5 mm) e grava `rotas_fios.json`, com a lista de conectores de jumper que batem no chassi |
| `rotas_fios.json` | pontos de passagem de cada fio; gerado, não editar |
| `montagem_src.html` | modelo da página, com as marcações que o gerador troca pelas peças e pelos dados |
| `ferramentas/gera_montagem.py` | monta a página, a lista de jumpers e a lista de interferências (`interferencias.json`, nesta pasta); para se o editor e o `config.h` discordarem na posição ou no ângulo de um sensor |
| `modelos/chassi/` | os STL do chassi do grupo, uma peça por arquivo (base, rampa, laterais, hastes, tampa, suportes dos motores, prateleira, calço em V e suportes de 160°) |
| `modelos/` | suporte do VL53L0X (`vl53l0x-bracket.stl`, do Cults3D, e o `.scad` de origem) e o módulo CJVL53L0XV2 no rebaixo dele (`cj_vl53l0x.stl`, do `ferramentas/gera_modulo_cj.py`) |

Referencial do robô: origem no meio do eixo das rodas, no chão, x para a frente, y para a esquerda
e z para cima, em mm. O editor tem a frente em -x, o eixo das rodas em x = 51 mm e o chão 3 mm
abaixo do z = 0 do desenho; o `exporta_pecas.py` converte para o referencial do robô
(`x = 51 - x`, `y = -y`, `z = z + 3`, giro de 180 graus em z, sem espelhar).

Para trocar o chassi:

```
python ferramentas/exporta_pecas.py robo_3d_editor.html
python ferramentas/gera_montagem.py          # confere o config.h contra o editor
python ferramentas/roteia_fios.py
python ferramentas/gera_montagem.py --interferencias
```

## Os dois andares

| Andar | O que fica nele |
|---|---|
| Base (z de 3 a 45 mm) | ponte H HW-956 (x de 30 a 71 mm), os dois motorredutores com as rodas, os quatro TCRT5000, os sete VL53L0X em seus suportes; entre os dois motores sobra só uma passagem de 5 mm (y de -2 a 3 mm), o único caminho dos fios entre a metade de trás e a da frente |
| Prateleira (placa em z de 31,2 a 32,8 mm, x de 17 a 71 mm, y de -37 a 38 mm, abas até y = ±51 mm) | extensora do ESP32 (z até 47,4 mm) e o ESP32 DevKit (z até 51,1 mm); dos lados da placa a prateleira é vazada, então os fios sobem da base pelas laterais |
| Tampa de trás (z de 46 a 47 mm, x de -26 a 15 mm) | furos A (chave DIP, x de 2 a 10 mm), B (botão de início, x de -8 a -2 mm) e C (chave KCD1, x de -22 a -12 mm); por cima, a célula 18650 da lógica (y de 6 a 75 mm) e o regulador TPS63070 |
| Tampa da frente (z de 53 a 55 mm, x de 16 a 77 mm) | as três células 18650 da bateria 3S (z de 55 a 74 mm), o BMS por cima (z até 78 mm) (sem protoboard) |

Não há protoboard. O SDA e o SCL dos sete VL53L0X se juntam em duas emendas soldadas com
termorretrátil (6 x 4 x 4 mm, em x = 17,5 mm, z de 39,5 a 43,5 mm), no vão atrás da fileira de
trás da extensora, cada uma com oito pontas: sete fios de sensor (jumper cortado ao meio, com a
fêmea no pino do sensor) e um jumper curto até o pino S do GPIO 21 ou 22. Todo o resto vai nos trios S, V e G da extensora, um trio por
GPIO: cada VL53L0X leva XSHUT, VIN e GND no trio do seu GPIO de XSHUT, e cada TCRT5000 leva A0,
VCC e GND no trio do seu GPIO (o D0 fica sem fio).

## Suporte dos sete VL53L0X

Os sete sensores de oponente usam o suporte do grupo (módulo CJVL53L0XV2 no rebaixo do suporte de
PLA): FE, FC e FD na face inclinada da frente (o FC em calço em V), LE e LD retos na lateral à
frente do eixo, TE e TD em suportes angulares de 160° atrás da roda. As posições e os ângulos saem
do editor e batem com a tabela `TOF` do `config.h` (o gerador confere, com 2 mm e 1 grau de
tolerância):

| Sensor | x (mm) | y (mm) | Ângulo | Altura da lente |
|---|---|---|---|---|
| LE, LD | 46 | ±70 | ±90° | 24,0 mm |
| FE, FD | 77 | ±32 a ±34 | ±17° | 23,9 mm |
| FC | 83 | 0 | 0° | 23,0 mm |
| TE, TD | -30 | ±66 | ±160° | 29,1 e 30,1 mm |

## Espaço no chassi e pendências

Medidas tiradas das malhas do editor (voxels de 1 mm na lista de interferências da página) e da
conferência dos conectores dos jumpers. Os itens 1 a 4 mudam o comportamento do robô e precisam de
decisão do projeto mecânico.

| # | Item | Medida | O que fazer |
|---|---|---|---|
| 1 | Inclinação dos VL53L0X da frente (FE, FC e FD) | os módulos ficam na rampa e apontam 27 a 28° para cima, e o firmware supõe feixe horizontal. A borda de baixo do cone (14,5° acima da horizontal) passa a 49 mm de altura a 100 mm de distância e a 101 mm a 300 mm | calço ou janela que deixe o módulo na horizontal, ou aceitar o alcance menor contra robôs baixos |
| 2 | Conectores dos pinos de trás da extensora (GPIO 23, 22, 1, 3, 21, 19, 18, 5, 17, 16, 4, 2, 15, 3V3 e GND) | o conector fêmea reto sai 8 mm acima da agulha (z = 44,6 mm); a cunha de apoio da tampa da frente (x de 17 a 25 mm, z de 48 a 53 mm) deixa 3,4 mm: os conectores S, V e G batem nela (61, 91 e 152 mm³) | rebaixo na cunha, tampa mais alta, ou conectores em ângulo reto. No modelo os pinos S, V e G de cada trio ficam a 1,7 mm um do outro, no módulo real são 2,54 mm: conferir |
| 3 | Passagem dos fios | as duas tampas não têm furo além dos três das chaves, e a prateleira deixa as laterais livres. Os fios que vão do andar de cima (células, BMS, regulador) para o de baixo passam pelo furo PF1 proposto (x de 17 a 25 mm, y de -5 a 3 mm, tampa da frente) ou dão a volta pela traseira aberta do chassi (x = -29 mm) | abrir o PF1 e um passa-fios na traseira; os fios pela traseira ficam expostos |
| 4 | Largura | 151 mm com os suportes de 160° (y = ±75,5 mm), contra 152 mm do gabarito da largada | só 1 mm de folga: não aumentar os suportes |
| 5 | TCRT5000 de trás (BTE e BTD) | face a x = -26,5 mm e y = ±69,4 mm, atrás da roda, com os pinos para dentro. A distância de parada na borda (38,3 mm) cabe na margem de 51,5 mm (25 mm da faixa mais 26,5 mm até o eixo), que o `static_assert` da cinemática confere | nenhuma, mas a folga é de 13 mm: mexer na velocidade ou no freio exige conferir de novo |
| 6 | TCRT5000 da frente (BFE e BFD) | face a x = 95,5 mm e y = ±62 mm, com a placa para dentro do chassi; o módulo encosta na base e na rampa (encaixe nos furos) | rasgo na rampa para o PCB, se a placa não passar |
| 7 | Chave KCD1 (furo C) e botão de início (furo B) | os dois corpos se sobrepõem 80 mm³ (flange da KCD1 contra o botão de 12 x 12 mm) | afastar o botão uns 2 mm ou diminuir a flange |
| 8 | Células da bateria 3S | os polos + das três células ficam do mesmo lado: as fitas de níquel entre elas atravessam 69 mm de célula, a menos que a célula 2 seja girada | girar a célula 2 no editor |
| 9 | Altura das lentes do TE e do TD | 29 e 30 mm, contra 23 a 24 mm das outras | só informativo (o `LENTE_ALTURA_MM` do `config.h` é a altura das lentes da frente e das laterais) |
| 10 | Modelos da extensora e do ESP32 | o editor usa uma placa de 44 pinos (22 fileiras de 2,23 mm); o DevKit de 30 pinos tem 15 pinos por lado, então os trios dos GPIOs foram postos nas 15 últimas fileiras, do lado do USB | trocar pelos modelos reais e rodar o `exporta_pecas.py` |

Posições dos pinos medidas nas malhas: a ordem dos pinos do VL53L0X (VIN, GND, SCL, SDA, GPIO1,
XSHUT) vem da foto da placa e supõe a malha sem espelho; os pinos da chave DIP, do botão e da
KCD1 foram achados pela forma das malhas (pernas embaixo da peça) e valem a posição, não a
numeração das pernas; o botão BOOT do DevKit tem posição aproximada (as duas pernas do botão de
início vão soldadas nas pernas do BOOT, e a ordem não importa).

## Posições dos componentes

As posições dos componentes são as do editor 3D; as dos sensores também estão no `config.h`, e o
gerador para se as duas fontes discordarem. A lista de interferências no fim da página (voxels de
1 mm, sobreposição de mais de 2 mm) mostra o que colide; os pares do módulo com o suporte, do
suporte com a parede da rampa e dos pinos do ESP32 na extensora são encaixes e aparecem marcados.
Os caminhos dos fios são calculados no volume das peças e não substituem a conferência da
montagem física: o fio passa por onde o modelo tem espaço livre de 3 mm.
