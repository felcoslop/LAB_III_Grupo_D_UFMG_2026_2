"""Converte as peças do STEP do robô em malhas compactas para a página 3D do combate.

Referencial de saída (mm): x para a frente, y para cima e z para a direita (o do three.js com o
robô olhando para +x), origem no meio do eixo das rodas, no chão. O STEP (LAB3-Modelagem, depois de
ferramentas/step_para_stl.py) tem a frente em -x, o eixo das rodas em x = 51 mm e o plano de
simetria em y = 0; o chão fica 3,0 mm abaixo do z = 0 do desenho (x_robô = 51 - x, y_robô = -y,
z_robô = z + 3,0).

Peças da página (uma malha por nome):
  base     base e as duas peças da frente (rampa)        corpo    laterais e tampa
  suporte  suportes dos motores e berços de PLA          abracE/D hastes laterais esquerda e direita
  motorE/D motorredutor JGB37-520 de cada lado, com o eixo e o acoplador
  pneuE/D  pneu StickyMAX 32 mm                          cuboE/D  cubo de PLA
E fica a esquerda em y_robô > 0.

Uso: python ferramentas/exporta_malhas.py PASTA_DOS_STL doc/combate/malhas.js
Precisa de: pip install trimesh fast-simplification networkx scipy
"""
import base64
import json
import os
import sys
import warnings

import numpy as np
import trimesh

warnings.filterwarnings("ignore")
D = sys.argv[1].rstrip("/") + "/"
EIXO_X, DZ = 51.0, 3.0

GRUPOS = {
    "base": ["Base", "Frente1", "Frente2"],
    "corpo": ["Lateral_L", "Lateral_R", "Topo"],
    "suporte": ["Suporte_L", "Suporte_R", "Body", "Body001", "COUPLING_BRACKET_GEARED_BODY_TO_DC_MOTOR_JGB37-520",
                "COUPLING_BRACKET_GEARED_BODY_TO_DC_MOTOR_JGB37-521"],
    "abracE": ["Haste_R"],     # no STEP o y é invertido: Haste_R do desenho fica em y_robô > 0
    "abracD": ["Haste_L"],
    "motorE": ["DC_MOTOR_JGB37-521", "PLASTIC_COVER_DC_MOTOR_JGB37-521", "CONNECTION_DC_MOTOR_JGB37-522",
               "CONNECTION_DC_MOTOR_JGB37-523", "MOTOR_SHAFT_JGB37-521", "SHAFT_COUPLER_JGB37-521"],
    "motorD": ["DC_MOTOR_JGB37-520", "PLASTIC_COVER_DC_MOTOR_JGB37-520", "CONNECTION_DC_MOTOR_JGB37-520",
               "CONNECTION_DC_MOTOR_JGB37-521", "MOTOR_SHAFT_JGB37-520", "SHAFT_COUPLER_JGB37-520"],
    "pneuE": ["StickyMAX-32mm-Pneu-Loja001"], "pneuD": ["StickyMAX-32mm-Pneu-Loja"],
    "cuboE": ["Roda_sumo501"], "cuboD": ["Roda_sumo500"],
}
ALVO = {"base": 4000, "corpo": 4000, "suporte": 5000, "motorE": 3500, "motorD": 3500, "pneuE": 2400, "pneuD": 2400,
        "cuboE": 2000, "cuboD": 2000}


def carrega(nome):
    m = trimesh.load(D + nome + ".stl", force="mesh")
    v = m.vertices
    # x_robô, y_robô = -y, z_robô; troca para (x, cima, -y_robô) do three.js
    xb, yb, zb = EIXO_X - v[:, 0], -v[:, 1], v[:, 2] + DZ
    return trimesh.Trimesh(np.c_[xb, zb, -yb], m.faces, process=True)


def junta(nomes):
    partes = [carrega(n) for n in nomes]
    m = trimesh.util.concatenate(partes)
    if m.volume < 0:
        m.invert()
    return m


pecas = {k: junta(v) for k, v in GRUPOS.items()}
for k, n in ALVO.items():
    if len(pecas[k].faces) > n:
        pecas[k] = pecas[k].simplify_quadric_decimation(face_count=n)
for k, m in pecas.items():
    print(f"{k:8s} tris {len(m.faces):6d} bounds {np.round(m.bounds, 1).tolist()}")
saida = {}
for k, m in pecas.items():
    sm = trimesh.graph.smooth_shade(m, angle=np.radians(35))
    v, n, f = sm.vertices, sm.vertex_normals, sm.faces.astype(np.uint32)
    q = np.round(v / 0.01).astype(np.int16)           # 0,01 mm
    nn = np.round(n * 127).astype(np.int8)
    ind = f.astype(np.uint16) if len(v) < 65536 else f
    saida[k] = {"p": base64.b64encode(q.tobytes()).decode(), "n": base64.b64encode(nn.tobytes()).decode(),
                "i": base64.b64encode(ind.tobytes()).decode(), "i32": len(v) >= 65536, "nv": int(len(v))}
txt = "window.MALHAS=" + json.dumps(saida, separators=(",", ":")) + ";\n"
open(sys.argv[2], "w").write(txt)
print("bytes", len(txt))
