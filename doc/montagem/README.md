# Montagem 3D: modelos, posições e espaço no chassi

Esta pasta gera a página `doc/site/montagem.html` (robô montado em 3D, com cada jumper). O
`README.md` da raiz explica a página; aqui ficam o fluxo de arquivos e a conferência de espaço no
chassi do projeto mecânico (STEP LAB3-Modelagem, outubro de 2026).

## Arquivos

| Arquivo | Para que serve |
|---|---|
| `step_para_stl.py` | separa o STEP do projeto mecânico em um STL por peça (precisa de `pip install cadquery-ocp`) |
| `exporta_pecas.py` | junta os STL do chassi e dos componentes em malhas compactas (`pecas.js`) |
| `montagem.json` | posição e giro de cada peça, pinos de cada modelo, ordem dos pinos do ESP32 na protoboard, uso da protoboard (I2C, 3V3 e GND) e caminho dos feixes de fios |
| `gera_montagem.py` | monta a página, a lista de jumpers e a lista de interferências (`interferencias.json`) |
| `modelos/` | suporte do VL53L0X (`vl53l0x-bracket.stl`, do Cults3D, e o `.scad` de origem) e o módulo CJVL53L0XV2 no rebaixo dele |

Referencial do robô: origem no meio do eixo das rodas, no chão, x para a frente, y para a esquerda
e z para cima, em mm. O STEP tem a frente em -x, o eixo das rodas em x = 51 mm e o chão 3 mm abaixo
do z = 0 do desenho; o `exporta_pecas.py` converte para o referencial do robô (`x = 51 - x`,
`y = -y`, `z = z + 3`).

Para trocar o chassi:

```
python doc/montagem/step_para_stl.py LAB3-Modelagem.step PASTA_DO_CHASSI
python doc/montagem/exporta_pecas.py PASTA_DO_CHASSI PASTA_DOS_COMPONENTES
python doc/montagem/gera_montagem.py --interferencias
```

## Suporte dos sete VL53L0X

Os sete sensores de oponente usam o suporte *Mounting Bracket for VL53L0X Module* do Cults3D: uma
placa de PLA de 32 x 20 x 3,5 mm com o rebaixo do módulo e dois furos de fixação. A lente fica na
altura `LENTE_ALTURA_MM` do `config.h`, e a posição e o ângulo de cada suporte saem da tabela `TOF`.
O suporte tem licença própria do autor; o STL não foi alterado, só movido e girado na página.

## Espaço no chassi

Medidas tiradas dos blocos de 1 mm do STEP (precisão de 0,5 mm) e das malhas dos componentes.

| Item | Medida | Folga ou problema |
|---|---|---|
| Corpo do chassi | x de -31 a 118 mm e y de -74,5 a 74,5 mm: 149 x 149 mm | cabe no gabarito de 15,2 x 15,2 cm da classe Lego/Vex de 1 kg, com 3 mm de sobra |
| Quatro células 18650 (3S mais a da lógica) | bloco de 2 x 2 em x de 27,8 a 65 mm, z de 12 a 49,2 mm | 3,8 mm até a face de baixo da tampa (z = 53 mm) |
| BMS 3S | x de 61,9 a 82,1 mm, deitado a z = 12 mm | cabe à frente do bloco das células |
| TPS63070 | em x = 44 mm, y = -50 mm, z = 12 mm | cabe ao lado do bloco, na caixa da direita |
| Ponte H HW-956 | na tampa de trás (z = 46,9 mm), em y = 45 mm, com a barra de sinais para trás | cabe |
| Chave DIP de 4 vias (11,8 x 10,4 mm) | no furo A da tampa de trás (x de -23 a -11 mm, y de -6 a 6 mm: 12 x 12 mm) | cabe, com 0,2 mm de folga em x e 1,6 mm em y: apertado, colar a chave com cola quente |
| Botão de ligar (mini gangorra de 15 x 10 mm, recorte de 13 x 9 mm) | no furo B da tampa de trás (x de -9,5 a -0,5 mm, y de -6,6 a 6,6 mm: 9,1 x 13,1 mm) | cabe: o furo é do tamanho desse recorte |
| Furo C da tampa de trás | x de 2 a 12 mm, y de -5,9 a 5,9 mm | livre: a página usa como passa-fios dos sensores de borda de trás e dos motores |
| Protoboard de 400 pontos (81 x 55 mm) com o ESP32 DevKit em cima | na tampa da frente (z = 55 mm), com os 81 mm em y e os 55 mm em x, de x = 16 a 71 mm | cabe: a tampa é plana de x = 16 a 77 mm no meio e se estreita para a ponta, então só um canto de 6 mm da protoboard passa da borda inclinada (cerca de 1,4% da área). O DevKit chega a z = 71 mm |
| Segunda protoboard de 81 x 55 mm | não tem lugar | a tampa de trás tem só 42 mm de comprimento (x de -26 a 15 mm) e a da frente já está ocupada; o que a segunda faria (barramentos de I2C, 3V3 e GND) cabe nas colunas 1 a 13 e nos trilhos da primeira |
| TCRT5000 da frente (BFE e BFD) | corpo de 38 mm, com o par infravermelho a 4 mm do chão e o PCB a 14 mm | o PCB atravessa a parede da rampa: **precisa de um rasgo** de 10 x 15 x 6 mm (x de 72 a 81 mm, y de ±55 a ±69 mm, z de 14 a 19 mm) em cada lado |
| TCRT5000 de trás (BTE e BTD) | atrás do motor, com os pinos para a frente e a ponta para fora | sai 32,6 mm atrás da base (x = -63,6 mm): o comprimento total vira 181,6 mm, **acima do gabarito de 15,2 cm** da classe |
| VL53L0X FE, FD e FC | módulos na face inclinada da rampa | precisam de janela na rampa: FE (x de 79 a 84 mm, y de 25 a 43 mm, z de 21 a 29 mm), FD (x de 75 a 79 mm, y de -42 a -39 mm, z de 25 a 29 mm) e FC (x de 85 a 92 mm, y de -7 a 0 mm, z de 26 a 29 mm) |
| VL53L0X LE e LD | janela entre os montantes da lateral, com o suporte colado por dentro | cabem; o módulo do LE encosta 91 mm³ no BFE (o LD não encosta no BFD): afastar o BFE 3 mm para a frente |
| VL53L0X TE e TD (retos, a ±90°) | janela da lateral de x = 22,4 a 42,4 mm (centro a 32,4 mm), suporte colado por dentro | cabem na largura do chassi (149 mm). **Pendência**: a janela do LE fica a 22 mm da do TE (centros em 54,4 e 32,4 mm) e o suporte tem 32 mm, então os dois suportes se sobrepõem 10 mm (945 mm³); o do TE não pode ir para trás (pneu) nem o do LE para a frente (rampa) |

O gabarito da classe de 1 kg (15,2 x 15,2 cm na largada) fica em 149 mm de largura, com os suportes de TE e TD por dentro da lateral; no comprimento ele passa por causa dos TCRT5000 de trás, que saem do chassi, a menos que a CoRA 2026 aceite sensor saliente. Saídas para a pendência dos suportes: afastar as duas janelas para 32 mm entre os centros (a do TE já está no limite do pneu, então a do LE iria para a rampa), ou cortar 10 mm do lado de dentro de cada suporte no `vl53l0x-bracket.scad` (o suporte perde um furo de parafuso; ele tem licença própria do autor).

## Posições dos componentes

Todas as posições de componentes dentro do chassi são provisórias e conferidas contra o STEP em
blocos de 1 mm, sem colisão. As posições dos sensores vêm do `config.h`. A lista de interferências
no fim da página (voxels de 1,5 mm, sobreposição de mais de 3 mm) mostra o que ainda colide; os
pares do suporte com o módulo, do suporte com a parede e dos pinos do ESP32 na protoboard
são encaixes, e aparecem marcados.
