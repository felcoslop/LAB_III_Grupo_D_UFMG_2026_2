#!/usr/bin/env python3
"""Monta a página da montagem 3D (doc/site/montagem.html) a partir do config.h e do montagem.json.

O config.h dá a posição de cada sensor (a mesma que o firmware usa) e o GPIO de cada ligação; o
montagem.json dá a posição dos outros componentes, os pinos de cada modelo, a ordem dos pinos da
placa de expansão, o uso da mini protoboard e o caminho dos feixes de fios; o pecas.js
(exporta_pecas.py) dá as malhas. Daqui sai, para a página: a matriz de cada peça, cada fio com o
tipo de jumper (macho-fêmea, fêmea-fêmea, macho-macho ou soldado), o caminho e o comprimento, a
lista de compras de jumpers e as interferências entre as peças (voxels de 1,5 mm).

Uso: python doc/montagem/gera_montagem.py [--saida doc/site/montagem.html] [--fragmento] [--interferencias]
     --interferencias refaz a lista de interferências (trimesh e scipy) e grava
     doc/montagem/interferencias.json; sem ele a página usa a lista gravada, e o CI confere que a
     página do repositório é igual à gerada.
"""
import argparse
import base64
import json
import math
import os
import re
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))

# ---------------------------------------------------------------- config.h
def le_config():
    txt = open(os.path.join(RAIZ, "config.h"), encoding="utf-8").read()
    defs = {}
    for linha in txt.splitlines():
        m = re.match(r"\s*#define\s+([A-Z_][A-Z0-9_]*)\s+(-?[0-9.]+)f?\b", linha)
        if m:
            defs.setdefault(m.group(1), []).append(float(m.group(2)))
    # as tabelas e as alturas do robô montado vêm no segundo bloco (#else do GEOMETRIA_BANCADA)
    blocos = re.findall(r"static const CfgToF TOF\[\] = \{(.*?)\n\};", txt, re.S)
    tof = []
    for nome, xs, end, x, y, ang in re.findall(
            r'\{\s*"(\w+)",\s*(\d+),\s*(0x[0-9A-Fa-f]+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)', blocos[-1]):
        tof.append({"nome": nome, "xshut": int(xs), "end": end, "x": float(x), "y": float(y), "ang": float(ang)})
    blocos = re.findall(r"static const CfgBorda BORDA\[\] = \{(.*?)\n\};", txt, re.S)
    borda = []
    for nome, pino, x, y, lado, frente in re.findall(
            r'\{\s*"(\w+)",\s*(\d+),\s*(-?\d+),\s*(-?\d+),\s*([-+]?\d+),\s*(true|false)', blocos[-1]):
        borda.append({"nome": nome, "pino": int(pino), "x": float(x), "y": float(y)})
    v = lambda k, i=-1: defs[k][i]
    return {"tof": tof, "borda": borda, "lente": v("LENTE_ALTURA_MM"), "borda_h": v("BORDA_ALTURA_MM"),
            "in": [int(v(f"PINO_IN{k}")) for k in range(1, 5)], "dip": [int(v(f"PINO_DIP{k}")) for k in range(1, 5)],
            "sda": int(v("PINO_SDA")), "scl": int(v("PINO_SCL")), "start": int(v("PINO_START"))}


# ---------------------------------------------------------------- geometria
# Contas em float do Python, sem numpy: o CI refaz a página e confere byte a byte com a do
# repositório, e o resultado não pode depender da biblioteca de álgebra da máquina.
def mmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(len(B))) for j in range(len(B[0]))] for i in range(len(A))]


def rot(r):
    rx, ry, rz = (math.radians(a) for a in (r or [0, 0, 0]))
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    Rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    Ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    Rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]
    return mmul(mmul(Rx, Ry), Rz)


def matriz(R, t):
    return [list(R[0]) + [t[0]], list(R[1]) + [t[1]], list(R[2]) + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def gira(M, d):
    return [M[i][0] * d[0] + M[i][1] * d[1] + M[i][2] * d[2] for i in range(3)]


def aplica(M, p):
    g = gira(M, p)
    return [g[i] + M[i][3] for i in range(3)]


def soma(a, b, k=1.0):
    return [a[i] + k * b[i] for i in range(3)]


def dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def arred(v, n):
    return [round(x + 0.0, n) + 0.0 for x in v]


def le_pecas_js():
    txt = open(os.path.join(AQUI, "pecas.js"), encoding="utf-8").read()
    return txt, json.loads(txt[txt.index("{"):txt.rindex("}") + 1])


def malha(dados):
    import numpy as np
    p = np.frombuffer(base64.b64decode(dados["p"]), dtype=np.int16).reshape(-1, 3) * 0.01
    i = np.frombuffer(base64.b64decode(dados["i"]), dtype=np.uint32 if dados["i32"] else np.uint16).reshape(-1, 3)
    return p.astype(float), i.astype(np.int64)


# ---------------------------------------------------------------- peças
def monta_pecas(cfg, conf):
    pecas, M = [], {}
    for pc in cfg["pecas"]:
        R = rot(pc.get("rot"))
        local = matriz(R, pc.get("pos", [0, 0, 0]))
        W = mmul(M[pc["pai"]], local) if pc.get("pai") else local
        M[pc["id"]] = W
        pecas.append(dict(pc, m=W))
    pin = cfg["pinos"]
    # VL53L0X na posição e no ângulo do config.h: lente na altura LENTE_ALTURA_MM, olhando para fora
    for s in conf["tof"]:
        a = math.radians(s["ang"])
        c, sn = math.cos(a), math.sin(a)
        R = [[-sn, 0.0, c], [c, 0.0, sn], [0.0, 1.0, 0.0]]      # colunas: x local, y local (para cima), lente
        lr = gira(matriz(R, [0, 0, 0]), pin["vl53l0x"]["lente"])
        t = [s["x"] - lr[0], s["y"] - lr[1], conf["lente"] - lr[2]]
        W = matriz(R, t)
        M[s["nome"]] = W
        pecas.append({"id": s["nome"], "modelo": "vl53l0x", "grupo": "sensores", "cor": "#2b7a3a",
                      "nome": f"VL53L0X {s['nome']} (XSHUT no GPIO {s['xshut']}, endereço {s['end']})", "m": W})
    # TCRT5000: face do par infravermelho na posição do config.h, a BORDA_ALTURA_MM do chão, com os
    # pinos para trás
    for s in conf["borda"]:
        R = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]   # giro de 180 graus em z
        fr = gira(matriz(R, [0, 0, 0]), pin["tcrt5000"]["face"])
        t = [s["x"] - fr[0], s["y"] - fr[1], conf["borda_h"] - fr[2]]
        W = matriz(R, t)
        M[s["nome"]] = W
        pecas.append({"id": s["nome"], "modelo": "tcrt5000", "grupo": "sensores", "cor": "#1f5fa8",
                      "nome": f"TCRT5000 {s['nome']} (A0 no GPIO {s['pino']})", "m": W})
    return pecas, M


# ---------------------------------------------------------------- pinos
class Pinos:
    def __init__(self, cfg, pecas, M):
        self.cfg, self.M = cfg, M
        self.modelo = {p["id"]: p["modelo"] for p in pecas}
        exp = cfg["placa_expansao"]
        self.exp = {}
        for lado in ("esquerda", "direita"):
            for k, rot_ in enumerate(exp[lado]):
                nome = rot_ if rot_ != "GND" else ("GND_E" if lado == "esquerda" else "GND_D")
                y = exp["y_primeira"] + exp["passo"] * k
                for c in "SVG":
                    self.exp[f"{c}{nome}"] = [exp[f"x_{c}"][lado], y, exp["z_pino"]]
        self.uso = cfg["protoboard_uso"]

    def ponto(self, ref):
        """'peça:pino' -> (posição, direção de saída do fio, tipo do ponto)."""
        pid, nome = ref.split(":", 1)
        W = self.M[pid]
        mod = self.modelo[pid]
        if mod == "expansao":
            return aplica(W, self.exp[nome]), gira(W, self.cfg["placa_expansao"]["d"]), "macho"
        if mod == "protoboard":
            lin, col = nome[0], int(nome[1:])
            linhas = "abcdefghij"
            i = linhas.index(lin)
            y = 13.97 - 2.54 * i if i < 5 else -3.81 - 2.54 * (i - 5)
            return aplica(W, [(col - 9) * 2.54, y, 8.5]), gira(W, [0, 0, 1]), "furo"
        p = self.cfg["pinos"][mod][nome]
        return aplica(W, p["p"]), gira(W, p["d"]), p["tipo"]


# ---------------------------------------------------------------- ligações
COR = {"3V3": "#d7372f", "GND": "#1b1b1b", "SDA": "#d81b9a", "SCL": "#ef7d1a", "XSHUT": "#2e9e44",
       "A0": "#1fbcc4", "DIP": "#2f6fd6", "IN1": "#8e44ad", "IN2": "#7b4a1e", "IN3": "#c9a227", "IN4": "#e6e6e6",
       "M+": "#2f9a3e", "M-": "#e3c62b", "BAT+": "#c62828", "BAT-": "#212121", "PONTE": "#f0f0f0", "NIQUEL": "#b9bcc0"}


def ligacoes(conf, cfg):
    L = []
    u = cfg["protoboard_uso"]

    def liga(de, para, tipo, rede, grupo, cor=None, rota=None, obs=""):
        L.append({"de": de, "para": para, "tipo": tipo, "rede": rede, "grupo": grupo,
                  "cor": cor or COR.get(rede.split()[0], "#888"), "rota": rota, "obs": obs})

    # furos livres de cada coluna da protoboard (a ponte entre as metades usa e e f)
    livres = {}

    def furo(col):
        fila = livres.setdefault(col, [f"{l}{col}" for l in "abcdghij"])
        return fila.pop(0)

    for col, rede in ((u["SDA"], "SDA"), (u["SCL"], "SCL")):
        liga(f"proto:e{col}", f"proto:f{col}", "MM", f"{rede} (ponte entre as metades)", "I2C", COR["PONTE"],
             obs="jumper curto por cima do canal do meio")
    for s in conf["tof"]:
        lado = "frente" if s["nome"] == "FC" else ("tras" if s["x"] < 0 else ("esquerda" if s["y"] > 0 else "direita"))
        g, n = s["xshut"], s["nome"]
        liga(f"{n}:VIN", f"exp:V{g}", "FF", "3V3", "Sensores de oponente", rota=lado, obs=f"V da barra do GPIO {g}")
        liga(f"{n}:GND", f"exp:G{g}", "FF", "GND", "Sensores de oponente", rota=lado, obs=f"G da barra do GPIO {g}")
        liga(f"{n}:XSHUT", f"exp:S{g}", "FF", f"XSHUT {n}", "Sensores de oponente", COR["XSHUT"], lado, f"GPIO {g}")
        liga(f"{n}:SDA", f"proto:{furo(u['SDA'])}", "MF", "SDA", "I2C", rota=lado, obs="coluna do SDA na protoboard")
        liga(f"{n}:SCL", f"proto:{furo(u['SCL'])}", "MF", "SCL", "I2C", rota=lado, obs="coluna do SCL na protoboard")
    liga(f"proto:{furo(u['SDA'])}", f"exp:S{conf['sda']}", "MF", "SDA", "I2C", obs=f"GPIO {conf['sda']}")
    liga(f"proto:{furo(u['SCL'])}", f"exp:S{conf['scl']}", "MF", "SCL", "I2C", obs=f"GPIO {conf['scl']}")

    for s in conf["borda"]:
        g, n = s["pino"], s["nome"]
        lado = "esquerda" if s["y"] > 0 else "direita"
        liga(f"{n}:VCC", f"exp:V{g}", "FF", "3V3", "Sensores de borda", rota=lado, obs=f"V da barra do GPIO {g}")
        liga(f"{n}:GND", f"exp:G{g}", "FF", "GND", "Sensores de borda", rota=lado, obs=f"G da barra do GPIO {g}")
        liga(f"{n}:A0", f"exp:S{g}", "FF", f"A0 {n}", "Sensores de borda", COR["A0"], lado,
             f"GPIO {g} (ADC1); o D0 fica sem fio")

    # chave DIP: um lado de cada via no 3V3 (coluna própria), o outro no GPIO
    c3 = u["3V3_DIP"]
    liga(f"proto:a{c3}", f"exp:V{conf['dip'][3]}", "MF", "3V3", "Chave DIP", obs="3,3 V para o comum da chave")
    for k, (col, g) in enumerate(zip(u["dip_colunas"], conf["dip"])):
        liga(f"proto:{'bcde'[k]}{c3}", f"proto:a{col}", "MM", "3V3", "Chave DIP", obs=f"lado de cima da via {k + 1}")
        liga(f"proto:j{col}", f"exp:S{g}", "MF", f"DIP {k + 1}", "Chave DIP", COR["DIP"],
             obs=f"GPIO {g} (pull-down interno: via ligada lê 1)")

    # ponte H
    liga("ponte:GND", "exp:G13", "FF", "GND", "Ponte H", obs="terra comum entre a lógica e os motores")
    for k, g in enumerate(conf["in"]):
        liga(f"ponte:IN{k + 1}", f"exp:S{g}", "FF", f"IN{k + 1}", "Ponte H", COR[f"IN{k + 1}"], obs=f"GPIO {g}")
    liga("ponte:OUT1", "motorE:M+", "S", "M+ esquerdo", "Motores", rota="tras", obs="fio 22 AWG soldado no motor e preso no borne")
    liga("ponte:OUT2", "motorE:M-", "S", "M- esquerdo", "Motores", rota="tras", obs="fio 22 AWG soldado no motor e preso no borne")
    liga("ponte:OUT3", "motorD:M+", "S", "M+ direito", "Motores", rota="tras", obs="fio 22 AWG soldado no motor e preso no borne")
    liga("ponte:OUT4", "motorD:M-", "S", "M- direito", "Motores", rota="tras", obs="fio 22 AWG soldado no motor e preso no borne")

    # bateria 3S e BMS
    liga("cel1:-", "bms:B-", "S", "BAT- B-", "Bateria 3S", COR["BAT-"])
    liga("cel1:+", "cel2:-", "S", "NIQUEL 1-2", "Bateria 3S", COR["NIQUEL"], obs="fita de níquel soldada")
    liga("cel2:+", "cel3:-", "S", "NIQUEL 2-3", "Bateria 3S", COR["NIQUEL"], obs="fita de níquel soldada")
    liga("cel1:+", "bms:B1", "S", "BAT+ B1", "Bateria 3S", "#e0a01e", obs="fio de equilíbrio")
    liga("cel2:+", "bms:B2", "S", "BAT+ B2", "Bateria 3S", "#e0a01e", obs="fio de equilíbrio")
    liga("cel3:+", "bms:B+", "S", "BAT+ B+", "Bateria 3S", COR["BAT+"])
    liga("bms:P+", "ponte:VCC", "S", "BAT+ 12 V", "Bateria 3S", COR["BAT+"], rota="potencia",
         obs="P+ do BMS no borne de alimentação (o esquema não tem chave neste ramo)")
    liga("bms:P-", "ponte:PGND", "S", "BAT- 0 V", "Bateria 3S", COR["BAT-"], rota="potencia", obs="P- do BMS no borne de alimentação")

    # alimentação da lógica: 18650, chave, TPS63070 em 3,3 V
    liga("cel_logica:+", "chave_liga:C", "S", "BAT+ lógica", "Lógica", COR["BAT+"], rota="potencia")
    liga("chave_liga:NA", "tps:VIN", "S", "BAT+ chaveado", "Lógica", COR["BAT+"], rota="potencia")
    liga("cel_logica:-", "tps:GNDI", "S", "BAT- lógica", "Lógica", COR["BAT-"])
    liga("tps:VOUT", "exp:S3V3", "SF", "3V3", "Lógica", rota="esquerda", obs="saída do TPS63070 em 3,3 V no pino 3V3 do DevKit")
    liga("tps:GNDO", "exp:SGND_D", "SF", "GND", "Lógica", rota="esquerda", obs="GND do TPS63070 no GND da placa")
    return L


# ---------------------------------------------------------------- caminho dos fios
def catmull(pts, n=10):
    P = [list(map(float, p)) for p in pts]
    P = [soma(soma(P[0], P[0]), P[1], -1)] + P + [soma(soma(P[-1], P[-1]), P[-2], -1)]
    out = []
    for k in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[k - 1], P[k], P[k + 1], P[k + 2]
        for s_ in range(n):
            t = s_ / n
            out.append([0.5 * ((2 * p1[i]) + (-p0[i] + p2[i]) * t + (2 * p0[i] - 5 * p1[i] + 4 * p2[i] - p3[i]) * t * t +
                               (-p0[i] + 3 * p1[i] - 3 * p2[i] + p3[i]) * t ** 3) for i in range(3)])
    out.append(P[-2])
    return out


COMP_CONECTOR = {"femea": 14.0, "macho": 9.0, "solda": 2.0, "borne": 6.0}


def ponta(tipo_jumper, lado, tipo_ponto):
    """Ponta do fio que encosta num ponto: fêmea num pino macho, macho num furo, solda ou borne."""
    if tipo_ponto == "macho":
        return "femea"
    if tipo_ponto == "furo":
        return "macho"
    return tipo_ponto


def caminhos(L, P, cfg):
    rotas = cfg["rotas"]
    for f in L:
        pa, da, ta = P.ponto(f["de"])
        pb, db, tb = P.ponto(f["para"])
        f["ponta_de"], f["ponta_para"] = ponta(f["tipo"], 0, ta), ponta(f["tipo"], 1, tb)
        ca, cb = COMP_CONECTOR[f["ponta_de"]], COMP_CONECTOR[f["ponta_para"]]
        # o fio sai do fim do conector, na direção de saída
        a0, b0 = soma(pa, da, ca), soma(pb, db, cb)
        alca = max(6.0, min(14.0, 0.2 * dist(a0, b0)))
        pts = [a0, soma(a0, da, alca)]
        if f["rota"]:
            pts += [list(map(float, w)) for w in rotas[f["rota"]]]
        pts += [soma(b0, db, alca), b0]
        curva = catmull(pts, 8)
        comp = sum(dist(curva[k + 1], curva[k]) for k in range(len(curva) - 1))
        f["pa"], f["da"], f["pb"], f["db"] = (arred(v, 2) for v in (pa, da, pb, db))
        f["pts"] = [arred(v, 1) for v in curva]
        f["mm"] = int(round(comp + ca + cb))


def jumper_padrao(mm):
    for c in (100, 150, 200, 300):
        if mm * 1.15 <= c:
            return c
    return int(math.ceil(mm * 1.15 / 50) * 50)


# ---------------------------------------------------------------- interferências
def interferencias(pecas, malhas, cfg, passo=1.5):
    import numpy as np
    ign = {p["id"] for p in cfg["pecas"] if p.get("ignora_interferencia") or p.get("visivel") is False}
    vox, caixa = {}, {}
    try:
        import trimesh
    except ImportError:
        return None
    proc = cfg["modelos"]
    for pc in pecas:
        if pc["id"] in ign:
            continue
        if pc["modelo"] in malhas:
            v, f = malhas[pc["modelo"]]
        elif "procedural" in proc.get(pc["modelo"], {}):
            mod = proc[pc["modelo"]]
            dim = np.array(mod["dim"], float)
            centro = np.array(mod.get("centro", [0, 0, dim[2] / 2]), float)
            bx = trimesh.creation.box(extents=dim)
            bx.apply_translation(centro)
            v, f = bx.vertices, bx.faces
        else:
            continue
        W = np.array(pc["m"], float)
        vw = v @ W[:3, :3].T + W[:3, 3]
        m = trimesh.Trimesh(vw, f, process=False)
        try:
            vg = m.voxelized(passo).fill()
            pts = vg.points
        except Exception:
            pts = vw
        idx = np.unique(np.floor(pts / passo).astype(np.int64), axis=0)
        vox[pc["id"]] = set(map(tuple, idx))
        caixa[pc["id"]] = (vw.min(0), vw.max(0))
    nomes = {p["id"]: p.get("nome", p["id"]) for p in pecas}
    grupo = {p["id"]: p.get("grupo") for p in pecas}
    modelo = {p["id"]: p["modelo"] for p in pecas}
    encaixe = [set(x) for x in cfg.get("encaixes_esperados", {}).get("pares", [])]
    pares = []
    ids = list(vox)
    esperados = {frozenset(x) for x in (("esp", "exp"), ("dip", "proto"))}
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            if frozenset((a, b)) in esperados or (grupo[a] == "chassi" and grupo[b] == "chassi"):
                continue
            (amin, amax), (bmin, bmax) = caixa[a], caixa[b]
            if np.any(amax < bmin) or np.any(bmax < amin):
                continue
            ov = vox[a] & vox[b]
            if not ov:
                continue
            arr = np.array(list(ov))
            # sobreposição de uma ou duas camadas de voxel é peça encostada na outra, não interferência
            if (arr.max(0) - arr.min(0) + 1).min() <= 2:
                continue
            vol = len(ov) * passo ** 3
            if vol >= 30:
                esp = any(e <= {a, b, modelo[a], modelo[b]} for e in encaixe)
                pares.append({"a": nomes[a], "b": nomes[b], "mm3": int(round(vol)), "encaixe": esp})
    pares.sort(key=lambda p: (p["encaixe"], -p["mm3"]))
    return pares


# ---------------------------------------------------------------- página
ESQUELETO = ('<!doctype html>\n<html lang="pt-BR">\n<head>\n<meta charset="utf-8">\n'
             '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n</head>\n<body>\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--saida", default=os.path.join(RAIZ, "doc", "site", "montagem.html"))
    ap.add_argument("--modelo", default=os.path.join(RAIZ, "doc", "montagem_src.html"))
    ap.add_argument("--interferencias", action="store_true",
                    help="refaz a conferência de interferências (precisa de trimesh e scipy) e grava doc/montagem/interferencias.json")
    ap.add_argument("--fragmento", action="store_true", help="sem <!doctype>, <head> e <body> (página publicada como artefato)")
    a = ap.parse_args()
    cfg = json.load(open(os.path.join(AQUI, "montagem.json"), encoding="utf-8"))
    conf = le_config()
    pecas_js, pecas_dados = le_pecas_js()
    pecas, M = monta_pecas(cfg, conf)
    P = Pinos(cfg, pecas, M)
    L = ligacoes(conf, cfg)
    caminhos(L, P, cfg)
    arq_inter = os.path.join(AQUI, "interferencias.json")
    if a.interferencias:
        malhas = {k: malha(v) for k, v in pecas_dados.items()}
        inter = interferencias(pecas, malhas, cfg)
        with open(arq_inter, "w", encoding="utf-8") as f:
            json.dump(inter, f, ensure_ascii=False, indent=1)
            f.write("\n")
    else:
        inter = json.load(open(arq_inter, encoding="utf-8")) if os.path.exists(arq_inter) else None

    # lista de compras de jumpers
    compras = {}
    for f in L:
        if f["tipo"] in ("FF", "MF", "MM"):
            c = jumper_padrao(f["mm"]) if f["tipo"] != "MM" or f["mm"] > 40 else 0
            chave = (f["tipo"], c)
            compras[chave] = compras.get(chave, 0) + 1
    lista = [{"tipo": t, "cm": c / 10 if c else None, "n": n} for (t, c), n in sorted(compras.items())]
    dados = {
        "sobre": cfg["sobre"], "provisorio": cfg["provisorio"],
        "modelos": {k: v for k, v in cfg["modelos"].items() if "procedural" in v},
        "pecas": [{"id": p["id"], "modelo": p["modelo"], "nome": p.get("nome", p["id"]), "grupo": p.get("grupo", ""),
                   "cor": p.get("cor", "#888"), "opacidade": p.get("opacidade", 1), "visivel": p.get("visivel", True),
                   "m": [round(p["m"][i][j] + 0.0, 4) + 0.0 for j in range(4) for i in range(4)]} for p in pecas],
        "fios": [{k: f[k] for k in ("de", "para", "tipo", "rede", "grupo", "cor", "obs", "ponta_de", "ponta_para",
                                    "pa", "da", "pb", "db", "pts", "mm")} for f in L],
        "placa": cfg["placa_expansao"], "proto": cfg["protoboard_uso"],
        "compras": lista, "interferencias": inter, "diferencas": cfg["esquematico"]["diferencas"],
        "pinos_firmware": {"tof": conf["tof"], "borda": conf["borda"], "in": conf["in"], "dip": conf["dip"],
                           "sda": conf["sda"], "scl": conf["scl"], "start": conf["start"]},
    }
    html = open(a.modelo, encoding="utf-8").read()
    for marca in ("/*PECAS*/", "/*MONTAGEM*/"):
        if html.count(marca) != 1:
            raise SystemExit(f"marca {marca} não aparece uma vez só no modelo")
    html = html.replace("/*PECAS*/", pecas_js.strip())
    html = html.replace("/*MONTAGEM*/", json.dumps(dados, ensure_ascii=False, separators=(",", ":")))
    if not a.fragmento:
        html = ESQUELETO + html + "\n</body>\n</html>\n"
    os.makedirs(os.path.dirname(os.path.abspath(a.saida)), exist_ok=True)
    with open(a.saida, "w", encoding="utf-8") as f:
        f.write(html)
    n = {t: sum(1 for f in L if f["tipo"] == t) for t in ("FF", "MF", "MM", "SF", "S")}
    print(f"{a.saida}: {len(html) // 1024} KB, {len(L)} ligações {n}")
    if inter:
        print("interferências:")
        for p in inter:
            print(f"  {p['mm3']:6d} mm³  {p['a']}  x  {p['b']}{'  (encaixe)' if p['encaixe'] else ''}")


if __name__ == "__main__":
    main()
