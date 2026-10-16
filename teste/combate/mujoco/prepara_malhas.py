"""Casca convexa de cada peça do chassi (referencial do robô, em mm) para a física do MuJoCo.

O MuJoCo só colide malhas convexas, então cada peça maciça do chassi do STEP (LAB3-Modelagem, depois
de doc/montagem/step_para_stl.py) vira a sua casca convexa. Os motores, as caixas de redução e os
suportes ficam de fora: estão dentro do volume das laterais e da base, e os pneus são cilindros
criados no código (fisica_mujoco.cpp). Referencial de saída: o do config.h (x para a frente, y para
a esquerda, z para cima, origem no meio do eixo das rodas, no chão); no STEP o chão fica 3,0 mm
abaixo do z = 0 e a frente aponta para -x (x_robô = 51 - x, y_robô = -y, z_robô = z + 3,0).

Uso: python teste/combate/mujoco/prepara_malhas.py PASTA_DOS_STL teste/combate/mujoco/malhas
Precisa de: pip install trimesh scipy
"""
import os
import sys

import numpy as np
import trimesh

PECAS = {"base": ["Base"], "rampa1": ["Frente1"], "rampa2": ["Frente2"], "topo": ["Topo"],
         "latL": ["Lateral_L"], "latR": ["Lateral_R"]}


def main():
    origem, saida = sys.argv[1].rstrip("/"), sys.argv[2].rstrip("/")
    os.makedirs(saida, exist_ok=True)
    for nome, arquivos in PECAS.items():
        partes = []
        for a in arquivos:
            m = trimesh.load(f"{origem}/{a}.stl", force="mesh")
            v = m.vertices.copy()
            m.vertices = np.c_[51.0 - v[:, 0], -v[:, 1], v[:, 2] + 3.0]
            partes.append(m)
        casca = trimesh.util.concatenate(partes).convex_hull
        casca.export(f"{saida}/{nome}.stl")
        print(nome, len(casca.faces), "faces", np.round(casca.bounds, 1).tolist())


if __name__ == "__main__":
    main()
