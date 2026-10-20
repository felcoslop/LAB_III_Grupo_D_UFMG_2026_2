# CAD final do robô: STEP dividido em um STL por objeto

O projeto mecânico entregou o STEP final da montagem (`LAB3-Modelagem_2.step`, 48,5 MB). A árvore dele tem 25 objetos no primeiro nível, na mesma ordem e com os mesmos nomes da árvore do FreeCAD: Base, Frente1, Frente2, Lateral_R, Lateral_L, Roda_Motor_D, Roda_Motor_L, Haste_L, Haste_R, Topo, Suporte_R, Suporte_L, os quatro Linha_*, os sete suportes de sensor SS_*, a extensora do ESP32 e o ESP32. Esta pasta guarda um STL para cada um desses objetos (9.7 MB no total). O FreeCAD mostra `Linha_L011` e parecidos porque acrescenta um número ao abrir o STEP; os arquivos usam o nome que o objeto tem no STEP.

Uma montagem (`Roda_Motor_D`, a extensora, o ESP32) ou uma peça de vários sólidos (`Linha_L`, com 11) sai em um arquivo só, como aparece na árvore. O `ferramentas/step_para_stl.py` faz o contrário e abre a montagem em todas as peças soltas, até os parafusos.

O STEP em si não fica no repositório. Para refazer os STL:

```
pip install cadquery-ocp trimesh numpy
python ferramentas/step_por_objeto.py LAB3-Modelagem_2.step cad/stl
```

## Referencial

Cada STL fica na posição e no referencial do próprio CAD: frente do robô em -x, eixo das rodas em x = 51 mm, plano de simetria em y = 0 e base em z = 0. Importados juntos no FreeCAD, no Blender ou no fatiador, os 25 arquivos voltam a formar o robô montado, sem matriz. Para o referencial do robô do `config.h` (origem no meio do eixo das rodas, no chão, x para a frente, y para a esquerda), vale `x_robô = 51 - x`, `y_robô = -y` e `z_robô = z + 3`. Nos nomes, R é o lado direito (y positivo no CAD) e L o esquerdo; em `Roda_Motor_D` e `Roda_Motor_L` a letra D é direita.

## Dimensões do conjunto

Com os 25 STL juntos, no referencial do robô, o conjunto vai de x = -31,0 a 118,0 mm (149 mm de comprimento, com a ponta da rampa em 118 mm), de y = -74,5 a 74,5 mm (149 mm de largura) e de z = 0 a 55 mm. Cabe no quadrado de 152 mm da classe com 3 mm de folga em cada eixo; no editor 3D v11 a largura era de 151 mm, com 1 mm de folga.

## Peças para impressão 3D (17)

O STEP não diz o material. As dez primeiras seguem o PLA do chassi do grupo, e os sete suportes SS_* são os suportes de sensor do projeto mecânico. O volume somado é de 189.5 cm³ (cerca de 235 g de PLA maciço, com 1,24 g/cm³; a massa real depende do preenchimento), o que ajuda a fechar o limite de 1000 g do robô.

| Arquivo | O que é | Caixa no referencial do CAD (mm) | Triângulos | Volume (cm³) |
|---|---|---|---|---|
| `Base.stl` | base do chassi | x -67,0 a 82,0; y -74,5 a 74,5; z -0,0 a 24,5 | 5378 | 98,08 |
| `Frente1.stl` | placa da rampa da frente, lado direito | x -49,4 a -0,8; y -0,0 a 74,5; z 6,0 a 50,0 | 48 | 6,78 |
| `Frente2.stl` | placa da rampa da frente, lado esquerdo | x -49,4 a -0,8; y -74,5 a 0,0; z 6,0 a 50,0 | 48 | 6,79 |
| `Lateral_R.stl` | lateral direita | x -23,5 a 76,7; y 72,5 a 74,5; z 9,0 a 43,9 | 124 | 2,49 |
| `Lateral_L.stl` | lateral esquerda | x -23,2 a 76,7; y -74,5 a -72,5; z 8,9 a 43,9 | 120 | 2,48 |
| `Haste_R.stl` | haste lateral direita | x 28,7 a 76,7; y 38,5 a 72,5; z 8,8 a 42,7 | 374 | 6,31 |
| `Haste_L.stl` | haste lateral esquerda | x 28,7 a 76,7; y -72,5 a -38,5; z 8,8 a 42,7 | 374 | 6,31 |
| `Topo.stl` | tampa superior | x -25,8 a 76,7; y -74,5 a 74,5; z 42,7 a 52,0 | 560 | 28,11 |
| `Suporte_R.stl` | suporte do motor direito | x 24,5 a 77,5; y 9,0 a 30,0; z 9,0 a 40,7 | 780 | 13,76 |
| `Suporte_L.stl` | suporte do motor esquerdo | x 24,5 a 77,5; y -31,0 a -10,0; z 9,0 a 40,7 | 780 | 13,76 |
| `SS_FM.stl` | suporte do VL53L0X FC (frente, meio) | x -34,5 a -31,0; y -13,0 a 13,0; z 8,0 a 24,5 | 1556 | 0,56 |
| `SS_FL.stl` | suporte do VL53L0X FE (frente, esquerda) | x -30,9 a -19,9; y -45,2 a -20,0; z 8,0 a 24,5 | 1556 | 0,56 |
| `SS_FR.stl` | suporte do VL53L0X FD (frente, direita) | x -31,0 a -20,0; y 20,1 a 45,2; z 8,0 a 24,5 | 1556 | 0,56 |
| `SS_L.stl` | suporte do VL53L0X LE (lateral esquerda) | x -16,4 a 9,6; y -72,5 a -69,0; z 8,0 a 24,5 | 1556 | 0,56 |
| `SS_R.stl` | suporte do VL53L0X LD (lateral direita) | x -16,4 a 9,6; y 69,0 a 72,5; z 8,0 a 24,5 | 1556 | 0,56 |
| `SS_TL.stl` | suporte do VL53L0X TE (trás, esquerda) | x 73,9 a 81,6; y -16,7 a -3,3; z 6,0 a 38,0 | 1556 | 0,90 |
| `SS_TR.stl` | suporte do VL53L0X TD (trás, direita) | x 73,9 a 81,6; y 2,3 a 15,7; z 6,0 a 38,0 | 1556 | 0,90 |

## Componentes comprados, só de referência (8)

| Arquivo | O que é | Caixa no referencial do CAD (mm) | Triângulos | Volume (cm³) |
|---|---|---|---|---|
| `Roda_Motor_D.stl` | motorredutor JGB37-520 com a roda StickyMAX de 32 mm, lado direito (montagem) | x 32,5 a 69,5; y 2,1 a 73,5; z -3,0 a 38,5 | 21846 | 28,29 |
| `Roda_Motor_L.stl` | motorredutor JGB37-520 com a roda StickyMAX de 32 mm, lado esquerdo (montagem) | x 32,5 a 69,5; y -74,5 a -3,1; z -3,0 a 38,5 | 21838 | 28,29 |
| `Linha_LF.stl` | módulo TCRT5000 BFE, frente esquerda (11 sólidos) | x -23,9 a 14,2; y -68,8 a -54,4; z -0,0 a 15,4 | 6986 | 0,98 |
| `Linha_RF.stl` | módulo TCRT5000 BFD, frente direita (11 sólidos) | x -23,9 a 14,2; y 54,5 a 69,0; z -0,0 a 15,4 | 6986 | 0,98 |
| `Linha_L.stl` | módulo TCRT5000 BTE, trás esquerda (11 sólidos) | x 67,5 a 82,0; y -74,5 a -36,4; z -0,0 a 15,4 | 6986 | 0,98 |
| `Linha_R.stl` | módulo TCRT5000 BTD, trás direita (11 sólidos) | x 67,5 a 82,0; y 36,4 a 74,5; z -0,0 a 15,4 | 6986 | 0,98 |
| `expansion_board_esp_32_shield_CAD211.stl` | extensora do ESP32 de 30 pinos (montagem) | x -25,6 a 27,8; y -38,8 a 32,9; z 22,2 a 37,1 | 60074 | 9,18 |
| `esp32_wroom_32D_kitV1.stl` | ESP32 DevKit V1 com o ESP32-WROOM-32D (montagem) | x -13,3 a 15,3; y -38,1 a 15,2; z 28,4 a 42,0 | 51918 | 2,89 |

O STEP não traz a ponte H HW-956, o BMS e as três células da bateria 3S, a célula 18650 da lógica, o regulador TPS63070, a chave KCD1, o botão de início, a chave DIP, os módulos VL53L0X nem os fios. Esses itens estão no editor 3D v11 e na página `montagem.html`.

## Diferenças entre o CAD final e o config.h

A tabela compara a tabela `TOF` e a tabela `BORDA` do robô montado (`GEOMETRIA_BANCADA 0`) com o STEP final, no referencial do robô. Nos suportes SS_*, a posição é o centro do suporte e o ângulo é o da normal da placa de 3,5 mm; o módulo dentro do rebaixo pode ficar alguns milímetros fora desse centro. Nos módulos Linha_*, a posição é o centro do sensor (o par infravermelho na ponta da placa).

| Sensor | `config.h` (x, y, ângulo) | CAD final (x, y, ângulo) | Diferença |
|---|---|---|---|
| FC | 83, 0, 0° | 84,4; 0,0; 0° | 1,4 mm em x |
| FE | 77, 32, 17° | 76,4; 32,6; 20° | 3° |
| FD | 77, -34, -17° | 76,5; -32,7; -20° | 3° e 1,3 mm em y |
| LE | 46, 70, 90° | 54,4; 71,4; 90° | 8,4 mm em x |
| LD | 46, -70, -90° | 54,4; -71,4; -90° | 8,4 mm em x |
| TE | -30, 66, 160° | -26,8; 10,0; 160° | 56 mm em y |
| TD | -30, -66, -160° | -26,8; -9,0; -160° | 57 mm em y |
| BFE | 96, 62 | 71,4; 61,6 | 24,6 mm em x |
| BFD | 96, -62 | 71,4; -61,7 | 24,6 mm em x |
| BTE | -27, 69 | -23,8; 71,0 | 3,2 mm em x |
| BTD | -27, -69 | -23,8; -71,0 | 3,2 mm em x |

Três diferenças pedem decisão do projeto, porque mexem no firmware e nas páginas:

- **Sensores de trás (TE e TD):** no CAD final ficam juntos no centro da traseira (y de cerca de ±10 mm), a ±160°, e não atrás das rodas (y = ±66 mm). O rastreador e a cobertura do par de trás foram estudados com a posição antiga.
- **Sensores de borda da frente:** no CAD final ficam a 71,4 mm do eixo, e não a 96 mm. A fuga da borda e a distância de parada dependem dessa medida.
- **Sensores de borda de trás:** a 23,8 mm do eixo, 2,7 mm mais perto que os 26,5 mm de `BORDA_ATE_RODA_MM`, margem que o `static_assert` da cinemática confere.

As placas dos sete suportes SS_* são verticais (elevação de 0°), o que resolve a pendência dos três sensores da frente a 27° para cima do chassi anterior, desde que o módulo fique paralelo à placa. O `config.h`, o `doc/montagem/` e a página `montagem.html` continuam com o chassi do editor 3D v11 até essa decisão.

## Conferência das malhas

- As caixas das malhas coincidem com as do CAD com até 0,03 mm de diferença.
- As 16 peças de impressão além da `Base` e os quatro `Linha_*` têm malha fechada.
- A `Base` tem uma aresta de 1 mm (x = -11,65 mm, y = -68,85 mm, z de 8 a 9 mm, no referencial do CAD) compartilhada por 4 faces no próprio STEP. A malha não tem buraco, o volume é válido (98,08 cm³) e os fatiadores costumam reparar sozinhos esse tipo de aresta, mas vale o projeto mecânico revisar a peça.
- As montagens (`Roda_Motor_D`, `Roda_Motor_L`, extensora e ESP32) têm sólidos que se tocam, então a verificação de malha fechada falha; o volume delas é a soma dos sólidos.
- Desvio da malha: 0,02 mm nas peças de impressão e 0,05 mm nas montagens compradas. O `indice.json` guarda a caixa, os triângulos, o volume e o tamanho de cada arquivo.
