#!/usr/bin/env python3
"""Converte os STL do chassi e dos componentes em malhas compactas para a página da montagem.

Cada modelo do bloco "modelos" do montagem.json vira uma malha no referencial dela, em mm, com
z para cima: as peças do chassi vão para o referencial do robô (origem no meio do eixo das
rodas, no chão, x para a frente e y para a esquerda) e os componentes ficam no referencial do
próprio STL (o montagem.json posiciona cada um). Os modelos "procedural" (protoboard, caixas)
são desenhados pela página e não passam por aqui.

Referencial "stl_robo": STL do projeto mecânico, com a frente em -x, o eixo das rodas em
x = 56,18 mm e z = 14,43 mm e o plano de simetria em y = -0,5 mm (o mesmo do
doc/combate/exporta_malhas.py). Referencial "roda": cubo ou pneu com o eixo no x ou no z local,
levado para o eixo das rodas no lado pedido.

Uso: python doc/montagem/exporta_pecas.py PASTA_DO_CHASSI PASTA_DOS_COMPONENTES [saída]
     (saída padrão: doc/montagem/pecas.js). Os nomes dos arquivos ficam no montagem.json; um STL
     com o mesmo nome e outro prefixo também serve (por exemplo 5b8667f1-Base-Sumo.stl).
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
EIXO_X, MEIO_Y, DZ = 56.18, -0.5, 15.97 - 14.43
R_RODA = 15.97


def acha(pasta, nome):
    """Arquivo da pasta com esse nome no fim (aceita prefixo, sem diferença de maiúsculas)."""
    alvo = nome.lower()
    for g in sorted(glob.glob(os.path.join(pasta, "*"))):
        if os.path.basename(g).lower().endswith(alvo):
            return g
    raise SystemExit(f"não achei {nome} em {pasta}")


def stl_robo(m, espelha=False, desloca=(0, 0, 0)):
    v = m.vertices + np.array(desloca, dtype=float)
    if espelha:
        v[:, 1] = 2 * MEIO_Y - v[:, 1]
    xb = EIXO_X - v[:, 0]
    yb = -(v[:, 1] - MEIO_Y)
    zb = v[:, 2] + DZ
    f = m.faces[:, ::-1] if espelha else m.faces
    t = trimesh.Trimesh(np.c_[xb, yb, zb], f, process=True)
    return t


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
    pastas = {"chassi": sys.argv[1], "componentes": sys.argv[2]}
    saida = sys.argv[3] if len(sys.argv) > 3 else os.path.join(AQUI, "pecas.js")
    cfg = json.load(open(os.path.join(AQUI, "montagem.json"), encoding="utf-8"))
    pecas = {}
    for nome, mod in cfg["modelos"].items():
        if "arquivo" not in mod:
            continue
        grupo, arq = mod["arquivo"].split("/", 1)
        m = trimesh.load(acha(pastas[grupo], arq), force="mesh")
        ref = mod.get("referencial", "local")
        if ref == "stl_robo":
            m = stl_robo(m, mod.get("espelha", False), mod.get("desloca_stl", (0, 0, 0)))
        elif ref == "roda":
            m = roda(m, mod["eixo_local"], mod["lado"], mod["y"])
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
