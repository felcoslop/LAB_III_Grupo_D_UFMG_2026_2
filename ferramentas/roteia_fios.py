#!/usr/bin/env python3
"""Acha o caminho de cada fio da montagem por dentro do chassi e grava doc/montagem/rotas_fios.json.

Cada peça do editor 3D vira um volume de voxels de 1,5 mm (as mesmas malhas e matrizes do gera_montagem.py),
os passa-fios propostos (peças do grupo "passafios") abrem um furo nesse volume, e cada fio é roteado
em 3D, do primeiro trecho reto de uma ponta ao da outra, por um caminho de custo mínimo que passa só por
voxels livres (com folga de um voxel para o fio) e prefere ficar longe das paredes e junto de fios
já roteados (feixes). Fio sem caminho livre é listado: ali falta um passa-fios ou o chassi está fechado.

Uso: python ferramentas/roteia_fios.py [--passo 1.5]
Precisa de: pip install trimesh scipy numpy scikit-image
"""
import argparse
import heapq
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gera_montagem as G                                   # noqa: E402

ORIGEM = np.array([-45.0, -90.0, 0.0])
FIM = np.array([135.0, 90.0, 96.0])


def abre_passa_fios(ocup, pecas, cfg, passo):
    """Os passa-fios propostos (peças do grupo passafios) abrem um furo no volume."""
    shape = ocup.shape
    for pc in pecas:
        if pc.get("grupo") != "passafios":
            continue
        dim = np.array(cfg["modelos"][pc["modelo"]]["dim"], float)
        c = np.array([pc["m"][0][3], pc["m"][1][3], pc["m"][2][3]])
        lo = np.floor((c - dim / 2 - ORIGEM) / passo).astype(int)
        hi = np.ceil((c + dim / 2 - ORIGEM) / passo).astype(int)
        ocup[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] = False
    return ocup


def casca_e_feixes(ocup, pecas, cfg, passo):
    """Volumes proibidos só para o fio: fora do contorno do chassi (menos acima das tampas) e dentro do cone de cada VL53L0X."""
    shape = np.array(ocup.shape)
    X, Y, Z = (ORIGEM[k] + (np.arange(shape[k]) + 0.5) * passo for k in range(3))
    gx, gy, gz = np.meshgrid(X, Y, Z, indexing="ij")
    c = cfg["contorno"]
    fora = (gx < c["x_min"]) | (gx > c["x_max"]) | (np.abs(gy) > c["y"])
    ocup |= fora & (gz < c["z_tampa"])
    # as duas tampas sem furos (os da chave KCD1, do botão e da DIP já têm peça dentro): só os passa-fios abrem
    for t in c["tampas"]:
        ocup |= (gx >= t["x"][0]) & (gx <= t["x"][1]) & (np.abs(gy) <= c["y"]) & (gz >= t["z"][0]) & (gz <= t["z"][1])
    for pc in pecas:
        pm = cfg["pinos"].get(pc["modelo"], {})
        if "feixe" not in pm:
            continue
        W = np.array(pc["m"], float)
        lente = W[:3, :3] @ np.array(pm["lente"], float) + W[:3, 3]
        eixo = W[:3, :3] @ np.array(pm["feixe"], float)
        rel = np.stack([gx - lente[0], gy - lente[1], gz - lente[2]], axis=-1)
        ao_longo = rel @ eixo
        perp = np.linalg.norm(rel - ao_longo[..., None] * eixo, axis=-1)
        # cone de 25 graus de abertura, com 3 mm de raio já na lente, até 60 mm
        ocup |= (ao_longo > -2) & (ao_longo < 60) & (perp < 3.0 + np.maximum(ao_longo, 0) * np.tan(np.radians(12.5)))
    return ocup


def volumes(pecas, malhas, cfg, passo):
    import trimesh
    shape = tuple(np.ceil((FIM - ORIGEM) / passo).astype(int))
    ocup = np.zeros(shape, bool)
    por_peca = {}
    for pc in pecas:
        if pc.get("grupo") == "passafios":
            continue
        mod = cfg["modelos"].get(pc["modelo"], {})
        if pc["modelo"] in malhas:
            v, f = malhas[pc["modelo"]]
        elif "procedural" in mod:
            dim = np.array(mod["dim"], float)
            centro = np.array(mod.get("centro", [0, 0, dim[2] / 2]), float)
            bx = trimesh.creation.box(extents=dim)
            bx.apply_translation(centro)
            v, f = bx.vertices, bx.faces
        else:
            continue
        W = np.array(pc["m"], float)
        m = trimesh.Trimesh(v @ W[:3, :3].T + W[:3, 3], f, process=False)
        try:
            pts = m.voxelized(passo).fill().points
        except Exception:
            pts = m.vertices
        idx = np.floor((pts - ORIGEM) / passo).astype(int)
        ok = np.all((idx >= 0) & (idx < np.array(shape)), axis=1)
        v = np.zeros(shape, bool)
        v[tuple(idx[ok].T)] = True
        por_peca[pc["id"]] = v
        ocup |= v
    return ocup, por_peca


def rdp(pts, eps):
    """Ramer-Douglas-Peucker: menos pontos sem sair mais que eps do caminho."""
    pts = np.asarray(pts, float)
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    ab = b - a
    n = np.linalg.norm(ab)
    d = np.linalg.norm(np.cross(pts - a, ab), axis=1) / n if n > 1e-9 else np.linalg.norm(pts - a, axis=1)
    k = int(np.argmax(d))
    if d[k] > eps:
        return np.vstack([rdp(pts[:k + 1], eps)[:-1], rdp(pts[k:], eps)])
    return np.vstack([a, b])


def main():
    from scipy import ndimage
    from skimage.graph import route_through_array
    ap = argparse.ArgumentParser()
    ap.add_argument("--passo", type=float, default=1.5)
    ap.add_argument("--saida", default=os.path.join(G.DADOS, "rotas_fios.json"))
    a = ap.parse_args()
    passo = a.passo
    cfg, conf, pecas, M, P, L = G.carrega(com_rotas=False)
    _, dados = G.le_pecas_js()
    malhas = {k: G.malha(v) for k, v in dados.items()}
    ocup, por_peca = volumes(pecas, malhas, cfg, passo)
    cont = sum(v.astype(np.int8) for v in por_peca.values())   # quantas peças ocupam cada voxel
    ocup = abre_passa_fios(casca_e_feixes(ocup, pecas, cfg, passo), pecas, cfg, passo)
    cheio = ndimage.binary_dilation(ocup, structure=np.ones((3, 3, 3), bool))
    dist = ndimage.distance_transform_edt(~ocup) * passo
    base = 1.0 + 3.0 * np.exp(-dist / 4.0)
    shape = np.array(ocup.shape)
    idx = lambda p: tuple(np.clip(np.floor((np.asarray(p, float) - ORIGEM) / passo).astype(int), 0, shape - 1))
    centro = lambda i: ORIGEM + (np.asarray(i, float) + 0.5) * passo
    usado = np.zeros(ocup.shape, bool)
    rotas, falhas = {}, []
    for f in L:
        e = G.extremos(f, P)
        chave = G.chave_do_fio(f)
        if f["de"].startswith("proto:") and f["para"].startswith("proto:"):
            continue                                          # ponte curta entre as metades de uma coluna
        ia, ib = idx(e["a1"]), idx(e["b1"])
        c = np.inf
        # em torno das pontas o fio sai de dentro do conector: abre uma bola, onde andar custa mais; se a ponta
        # está encostada numa parede, a bola cresce até achar caminho
        for raio in (3, 4, 5, 6, 8, 10):
            custo = base.copy()
            custo[usado] *= 0.6
            livre = ~cheio
            for p in (e["a1"], e["b1"]):
                i = np.array(idx(p))
                lo, hi = np.maximum(i - raio, 0), np.minimum(i + raio + 1, shape)
                sub = (slice(lo[0], hi[0]), slice(lo[1], hi[1]), slice(lo[2], hi[2]))
                gx, gy, gz = np.ogrid[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]]
                bola = (gx - i[0]) ** 2 + (gy - i[1]) ** 2 + (gz - i[2]) ** 2 <= raio * raio
                custo[sub] = np.where(bola & cheio[sub], custo[sub] * 4.0, custo[sub])
                livre[sub] |= bola
            custo = np.where(livre, custo, np.inf)
            custo[ia] = custo[ib] = 1.0
            try:
                caminho, c = route_through_array(custo, ia, ib, fully_connected=True, geometric=True)
            except ValueError:
                c = np.inf
            if np.isfinite(c):
                break
        if not np.isfinite(c):
            falhas.append(chave)
            continue
        pts = np.array([centro(i) for i in caminho])
        pts[0], pts[-1] = e["a1"], e["b1"]
        for i in caminho:
            usado[i] = True
        s = rdp(pts, 1.0)
        rotas[chave] = [[round(float(x), 1) for x in p] for p in s[1:-1]]
        print(f"{chave:28s} {len(caminho):4d} voxels, {len(s) - 2} pontos")
    # conectores que batem no chassi: a caixa do conector (2,6 mm de lado) acima da ponta do pino, até onde o fio sai
    conflitos = []
    for f in L:
        e = G.extremos(f, P)
        for lado, ponto, d, ponta in (("de", e["pa"], e["da"], e["ponta_de"]), ("para", e["pb"], e["db"], e["ponta_para"])):
            if ponta not in ("femea", "macho"):
                continue
            ponto_id = f[lado == "de" and "de" or "para"].split(":")[0]
            d = np.asarray(d, float)
            u = np.cross(d, [1.0, 0.0, 0.0] if abs(d[0]) < 0.9 else [0.0, 1.0, 0.0])
            u /= np.linalg.norm(u)
            w = np.cross(d, u)
            vox = set()
            outras = (cont - por_peca[ponto_id].astype(np.int8)) > 0 if ponto_id in por_peca else cont > 0
            for t in np.arange(0.5, G.SAIDA[ponta], 0.75):
                for du in (-1.3, 0.0, 1.3):
                    for dw in (-1.3, 0.0, 1.3):
                        q = np.asarray(ponto, float) + d * t + u * du + w * dw
                        i = np.floor((q - ORIGEM) / passo).astype(int)
                        if np.all(i >= 0) and np.all(i < shape) and outras[tuple(i)]:
                            vox.add(tuple(i))
            if len(vox) * passo ** 3 >= 30:
                conflitos.append({"fio": G.chave_do_fio(f), "lado": lado, "ponto": f[lado == "de" and "de" or "para"],
                                  "mm3": int(round(len(vox) * passo ** 3))})
    with open(a.saida, "w", encoding="utf-8") as fo:
        json.dump({"_nota": "gerado por ferramentas/roteia_fios.py; não editar. Pontos de passagem (mm, referencial do robô) entre os dois primeiros trechos retos de cada fio, e conectores de jumper que batem em alguma peça do chassi.",
                   "rotas": rotas, "conectores_em_conflito": conflitos}, fo, ensure_ascii=False, indent=0)
        fo.write("\n")
    print(f"{len(rotas)} fios roteados em {a.saida}; {len(conflitos)} conectores batem no chassi")
    for c in conflitos:
        print(f"  {c['ponto']:16s} {c['mm3']:5d} mm³  (fio {c['fio']})")
    if falhas:
        print("sem caminho livre:")
        for k in falhas:
            print("  ", k)


if __name__ == "__main__":
    main()
