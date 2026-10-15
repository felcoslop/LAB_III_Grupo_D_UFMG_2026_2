#!/usr/bin/env python3
"""Gera o modelo simplificado do módulo CJVL53L0XV2 (16,5 x 13 mm) dentro do suporte de PLA.

O suporte (vl53l0x-bracket.stl, de TOBI, Cults3D "Mounting bracket for VL53L0X module") é uma placa
de 32 x 20 x 3,5 mm com um rebaixo de 16,5 x 13 mm para a placa do módulo, dois furos M3 nas pontas e
uma janela de 6 x 3 mm para o sensor, que olha pelo fundo da placa (a que encosta no chassi).

O módulo sai no referencial do suporte já girado (o mesmo que o exporta_pecas.py dá ao STL do
suporte): origem no centro da janela, na face de baixo da placa; +z é o feixe, para fora do chassi;
x ao longo dos 32 mm e y ao longo dos 20 mm, com y para cima quando o suporte está de pé.
Placa de 1,6 mm no rebaixo (z de -3,1 a -1,5), o chip de 4,4 x 2,5 x 1 mm voltado para a janela e a
barra de 6 pinos (VIN, GND, SCL, SDA, GPIO1, XSHUT) de passo 2,54 mm na ponta de x negativo, com os
pinos para trás (-z). Dimensões do chip e da barra conferidas no catálogo do módulo; conferir a
placa que chegar.

Uso: python doc/montagem/modelos/gera_modulo_cj.py   (grava cj_vl53l0x.stl ao lado)
Precisa de: pip install trimesh numpy
"""
import os

import numpy as np
import trimesh

AQUI = os.path.dirname(os.path.abspath(__file__))


def caixa(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return b


partes = [
    caixa(-8.25, 8.25, -4.25, 8.75, -3.1, -1.5),          # placa
    caixa(-2.2, 2.2, -1.25, 1.25, -1.5, -0.5),            # chip, com a lente na janela
    caixa(-8.0, -6.0, -4.45, 8.95, -5.6, -3.1),           # base de plástico da barra de pinos
]
for k in range(6):
    y = 2.25 + (k - 2.5) * 2.54
    partes.append(caixa(-7.32, -6.68, y - 0.32, y + 0.32, -12.0, -3.1))   # pino de 0,64 mm
m = trimesh.util.concatenate(partes)
m.export(os.path.join(AQUI, "cj_vl53l0x.stl"))
print("cj_vl53l0x.stl", len(m.faces), "triângulos", np.round(m.bounds, 2).tolist())
