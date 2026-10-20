#!/usr/bin/env python3
"""Converte as peças do editor 3D do chassi (robo_3d_editor_v11.html) em malhas compactas para a página da montagem.

O editor guarda cada peça já na posição final da montagem (triângulos soltos, int16 em décimos de
mm mais o pivô, uma cor por triângulo), no referencial do desenho: frente em -x, eixo das rodas em
x = 51 mm, base em z = 0 e chão em z = -3. A conversão para o referencial do robô (origem no meio
do eixo das rodas, no chão, x para a frente, y para a esquerda, z para cima, em mm) é
x_robo = 51 - x, y_robo = -y, z_robo = z + 3 (giro de 180 graus em z, sem espelhar).

Cada modelo do bloco "modelos" do montagem.json diz de quais peças do editor ele vem ("editor":
lista de índices, na ordem da lista ITEMS do arquivo), quantos triângulos ficam depois da
simplificação ("faces"), quanto o contorno pode mudar nela ("tol", em mm) e se leva a cor de
cada triângulo ("cores"). Peças idênticas (os sete
VL53L0X com suporte, os quatro TCRT5000, as células) viram um modelo só, no lugar da primeira
peça da lista; as outras entram na página com uma matriz de posição, achada aqui pelo ajuste de
corpo rígido de Kabsch entre os vértices das duas cópias ("editor_item" no bloco "pecas"). As
matrizes vão para doc/montagem/editor.json, que o gera_montagem.py lê.

Uso: python ferramentas/exporta_pecas.py ARQUIVO_DO_EDITOR.html [pecas.js] [editor.json]
     (saídas padrão: doc/montagem/pecas.js e doc/montagem/editor.json)
Precisa de: pip install trimesh fast-simplification scipy numpy
"""
import base64
import json
import os
import re
import sys
import warnings

import numpy as np
import trimesh
from scipy.spatial import cKDTree

warnings.filterwarnings("ignore")
AQUI = os.path.dirname(os.path.abspath(__file__))
DADOS = os.path.join(os.path.dirname(AQUI), "doc", "montagem")
EIXO_X, DZ = 51.0, 3.0


def le_itens(caminho):
    """Lista de peças do editor: vértices (triângulos soltos, referencial do robô) e cor de cada triângulo."""
    txt = open(caminho, encoding="utf-8").read()
    i = txt.index("const ITEMS=")
    j = txt.index("];", i)
    itens = json.loads(txt[i + len("const ITEMS="):j + 1])
    saida = []
    for it in itens:
        p = np.frombuffer(base64.b64decode(it["pos"]), dtype=np.int16).astype(float).reshape(-1, 3) * 0.1
        p = p + np.array(it["piv"], float)
        cor = np.frombuffer(base64.b64decode(it["col"]), dtype=np.uint8).reshape(-1, 3)
        v = np.c_[EIXO_X - p[:, 0], -p[:, 1], p[:, 2] + DZ]
        saida.append({"nome": it["name"], "v": v, "cor": cor})
    return saida


def kabsch(A, B):
    """Giro R e translação t com B ~ A R^T + t (corpo rígido, vértices correspondentes)."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cb - R @ ca


def malha_de(itens, idx):
    v = np.concatenate([itens[k]["v"] for k in idx])
    c = np.concatenate([itens[k]["cor"] for k in idx])
    m = trimesh.Trimesh(v, np.arange(len(v)).reshape(-1, 3), process=False)
    m.merge_vertices()
    return m, c


def simplifica(m, cores, alvo, tol=0.3):
    """Menos triângulos sem perder pedaços: se o contorno muda mais de tol mm (0,3 por padrão), a meta dobra até voltar."""
    if not alvo or alvo >= len(m.faces):
        return m, cores
    ref = cKDTree(m.triangles_center)
    while alvo < len(m.faces):
        s = m.simplify_quadric_decimation(face_count=alvo)
        if np.abs(s.bounds - m.bounds).max() <= tol:
            return s, cores[ref.query(s.triangles_center)[1]]
        alvo *= 2
    return m, cores


def compacta(m, cores, com_cor):
    """Vértices únicos por (posição, normal suavizada a 35 graus, cor), para a cor não escorrer entre triângulos."""
    sm = trimesh.graph.smooth_shade(m, angle=np.radians(35))
    cantos = sm.faces.ravel()
    q = np.round(sm.vertices[cantos] / 0.01).astype(np.int16)         # 0,01 mm, até +-327 mm
    n = np.round(sm.vertex_normals[cantos] * 127).astype(np.int8)
    cols = [q.astype(np.int32), n.astype(np.int32)]
    if com_cor:
        cols.append(np.repeat(cores, 3, axis=0).astype(np.int32))
    chave = np.concatenate(cols, axis=1)
    unico, ind = np.unique(chave, axis=0, return_inverse=True)
    ind = ind.reshape(-1).reshape(-1, 3)
    nv = len(unico)
    d = {"p": base64.b64encode(unico[:, 0:3].astype(np.int16).tobytes()).decode(),
         "n": base64.b64encode(unico[:, 3:6].astype(np.int8).tobytes()).decode()}
    if com_cor:
        d["c"] = base64.b64encode(unico[:, 6:9].astype(np.uint8).tobytes()).decode()
    ind = ind.astype(np.uint16) if nv < 65536 else ind.astype(np.uint32)
    d.update({"i": base64.b64encode(ind.tobytes()).decode(), "i32": nv >= 65536, "nv": int(nv),
              "min": np.round(sm.vertices.min(0), 2).tolist(), "max": np.round(sm.vertices.max(0), 2).tolist()})
    return d


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    saida = sys.argv[2] if len(sys.argv) > 2 else os.path.join(DADOS, "pecas.js")
    saida_poses = sys.argv[3] if len(sys.argv) > 3 else os.path.join(DADOS, "editor.json")
    cfg = json.load(open(os.path.join(DADOS, "montagem.json"), encoding="utf-8"))
    itens = le_itens(sys.argv[1])
    pecas, ref_do_modelo = {}, {}
    for nome, mod in cfg["modelos"].items():
        if "editor" not in mod:
            continue
        m, cores = malha_de(itens, mod["editor"])
        ref_do_modelo[nome] = mod["editor"][0]
        m, cores = simplifica(m, cores, mod.get("faces"), mod.get("tol", 0.3))
        pecas[nome] = compacta(m, cores, mod.get("cores", False))
        print(f"{nome:12s} {len(m.faces):6d} triângulos  {np.round(m.bounds, 1).tolist()}")
    poses = {}
    for pc in cfg["pecas"]:
        k = pc.get("editor_item")
        if k is None:
            continue
        r = ref_do_modelo[pc["modelo"]]
        A, B = itens[r]["v"], itens[k]["v"]
        if A.shape != B.shape:
            raise SystemExit(f"{pc['id']}: a peça {k} do editor não é cópia da peça {r}")
        R, t = kabsch(A, B)
        erro = np.linalg.norm(A @ R.T + t - B, axis=1).max()
        if erro > 0.4:
            raise SystemExit(f"{pc['id']}: o ajuste rígido deixou erro de {erro:.2f} mm (peça {k} contra a {r})")
        poses[pc["id"]] = {"R": np.round(R, 5).tolist(), "pos": np.round(t, 3).tolist(), "erro_mm": round(float(erro), 3)}
    with open(saida, "w") as f:
        f.write("window.PECAS=" + json.dumps(pecas, separators=(",", ":")) + ";\n")
    with open(saida_poses, "w", encoding="utf-8") as f:
        json.dump({"_nota": "gerado por ferramentas/exporta_pecas.py a partir do editor 3D; não editar",
                   "poses": poses}, f, indent=1)
        f.write("\n")
    print(f"{saida}: {os.path.getsize(saida) // 1024} KB; {saida_poses}: {len(poses)} poses")


if __name__ == "__main__":
    main()
