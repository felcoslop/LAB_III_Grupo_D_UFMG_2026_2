#!/usr/bin/env python3
"""Converte os STL do chassi e dos componentes em malhas compactas para a página da montagem.

Cada modelo do bloco "modelos" do montagem.json vira uma malha no referencial dela, em mm, com
z para cima: as peças do chassi vão para o referencial do robô (origem no meio do eixo das
rodas, no chão, x para a frente e y para a esquerda) e os componentes ficam no referencial do
próprio STL (o montagem.json posiciona cada um). Os modelos "procedural" (protoboard, caixas)
são desenhados pela página e não passam por aqui.

Referencial "step_robo": STL gerado do STEP do projeto mecânico (LAB3-Modelagem, por
doc/montagem/step_para_stl.py), com a frente em -x, o eixo das rodas em x = 51 mm e o plano de
simetria em y = 0; o chão fica 3,0 mm abaixo do z = 0 do desenho. O modelo pode juntar várias
peças ("arquivos"). Referencial "local_gira": o STL é movido para a origem dada e girado pela matriz
dada (usado no suporte do VL53L0X, para a lente ficar na origem e o feixe em +z). Referencial
"roda": cubo ou pneu do projeto antigo, com o eixo no x ou no z local.

Uso: python doc/montagem/exporta_pecas.py PASTA_DO_CHASSI PASTA_DOS_COMPONENTES [saída]
     (PASTA_DO_CHASSI é a saída do step_para_stl.py; saída padrão: doc/montagem/pecas.js). Os
     arquivos de cada modelo ficam no montagem.json, no grupo "chassi", "componentes" ou "repo" (a
     pasta doc/montagem/modelos).
Precisa de: pip install trimesh fast-simplification networkx scipy
"""
import base64
import glob
import json
import os
import sys
import warnings

import numpy as np
import trimesh

warnings.filterwarnings("ignore")
AQUI = os.path.dirname(os.path.abspath(__file__))
EIXO_X, DZ = 51.0, 3.0
R_RODA = 16.0


def acha(pasta, nome):
    """Arquivo da pasta com esse nome no fim (aceita prefixo, sem diferença de maiúsculas)."""
    alvo = nome.lower()
    arqs = sorted(glob.glob(os.path.join(pasta, "*")))
    for g in arqs:                                   # primeiro o nome exato
        if os.path.basename(g).lower() == alvo:
            return g
    for g in arqs:                                   # depois com prefixo
        if os.path.basename(g).lower().endswith(alvo):
            return g
    raise SystemExit(f"não achei {nome} em {pasta}")


def step_robo(m):
    """STL do STEP -> referencial do robô."""
    v = m.vertices
    return trimesh.Trimesh(np.c_[EIXO_X - v[:, 0], -v[:, 1], v[:, 2] + DZ], m.faces, process=True)


def local_gira(m, origem, R):
    v = (m.vertices - np.array(origem, float)) @ np.array(R, float).T
    f = m.faces if np.linalg.det(np.array(R, float)) > 0 else m.faces[:, ::-1]
    return trimesh.Trimesh(v, f, process=True)


def roda(m, eixo_local, lado, y):
    v = m.vertices
    if eixo_local == "x":
        a, b, c = v[:, 0], v[:, 1], v[:, 2]
    else:
        a, b, c = v[:, 2], v[:, 0], v[:, 1]
    t = trimesh.Trimesh(np.c_[b, lado * a + y, c + R_RODA], m.faces, process=True)
    if t.volume < 0:
        t.invert()
    return t


def simplifica(m, alvo):
    """Menos triângulos sem perder pedaços: peças com muitos pedaços soltos (pinos, abas) somem
    na simplificação; se o contorno muda mais de 0,3 mm, a meta dobra até voltar à malha original."""
    if not alvo:
        return m
    while alvo < len(m.faces):
        s = m.simplify_quadric_decimation(face_count=alvo)
        if np.abs(s.bounds - m.bounds).max() <= 0.3:
            return s
        alvo *= 2
    return m


def compacta(m):
    sm = trimesh.graph.smooth_shade(m, angle=np.radians(35))
    v, n, f = sm.vertices, sm.vertex_normals, sm.faces
    q = np.round(v / 0.01).astype(np.int16)      # 0,01 mm, até ±327 mm
    nn = np.round(n * 127).astype(np.int8)
    ind = f.astype(np.uint16) if len(v) < 65536 else f.astype(np.uint32)
    return {"p": base64.b64encode(q.tobytes()).decode(), "n": base64.b64encode(nn.tobytes()).decode(),
            "i": base64.b64encode(ind.tobytes()).decode(), "i32": len(v) >= 65536, "nv": int(len(v)),
            "min": np.round(v.min(0), 2).tolist(), "max": np.round(v.max(0), 2).tolist()}


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    pastas = {"chassi": sys.argv[1], "componentes": sys.argv[2], "repo": os.path.join(AQUI, "modelos")}
    saida = sys.argv[3] if len(sys.argv) > 3 else os.path.join(AQUI, "pecas.js")
    cfg = json.load(open(os.path.join(AQUI, "montagem.json"), encoding="utf-8"))
    pecas = {}
    for nome, mod in cfg["modelos"].items():
        if "arquivo" not in mod and "arquivos" not in mod:
            continue
        ms = []
        for ref_arq in mod.get("arquivos", [mod.get("arquivo")]):
            grupo, arq = ref_arq.split("/", 1)
            ms.append(trimesh.load(acha(pastas[grupo], arq), force="mesh"))
        m = ms[0] if len(ms) == 1 else trimesh.util.concatenate(ms)
        ref = mod.get("referencial", "local")
        if ref == "step_robo":
            m = step_robo(m)
        elif ref == "local_gira":
            m = local_gira(m, mod["origem"], mod["R"])
        elif ref == "roda":
            m = roda(m, mod["eixo_local"], mod["lado"], mod["y"])
        if mod.get("escala"):                        # ajuste de proporção do STL (ex.: DevKit com 22,86 mm entre as fileiras)
            m = trimesh.Trimesh(m.vertices * np.array(mod["escala"], float), m.faces, process=True)
        if m.volume < 0:
            m.invert()
        m = simplifica(m, mod.get("faces"))
        pecas[nome] = compacta(m)
        b = np.round(m.bounds, 1).tolist()
        print(f"{nome:12s} {len(m.faces):6d} triângulos  {b}")
    txt = "window.PECAS=" + json.dumps(pecas, separators=(",", ":")) + ";\n"
    open(saida, "w").write(txt)
    print(f"{saida}: {len(txt) // 1024} KB")


if __name__ == "__main__":
    main()
