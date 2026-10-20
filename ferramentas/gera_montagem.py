#!/usr/bin/env python3
"""Monta a página da montagem 3D (site/montagem.html) a partir do editor 3D do chassi, do config.h e do montagem.json.

O editor 3D (exporta_pecas.py) dá as malhas (pecas.js) e a matriz de cada peça repetida (editor.json);
o montagem.json dá os pinos de cada modelo, as emendas do barramento de I2C e os
passa-fios propostos; o config.h dá o GPIO de cada ligação e a tabela de posições dos sensores, que
precisa bater com a posição e o ângulo de cada sensor no editor (a conferência para a geração se
a diferença passar de 2 mm ou 1 grau). Daqui sai, para a página: a matriz de cada peça, cada fio com
o tipo de jumper (derivado das duas pontas: fêmea em pino macho, macho em furo, solda ou borne),
o caminho (doc/montagem/rotas_fios.json, feito por ferramentas/roteia_fios.py) e o comprimento, a lista
de compras de jumpers e as interferências entre as peças (voxels de 1,5 mm).

Uso: python ferramentas/gera_montagem.py [--saida site/montagem.html] [--fragmento] [--interferencias]
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
RAIZ = os.path.dirname(AQUI)
DADOS = os.path.join(RAIZ, "doc", "montagem")

# ---------------------------------------------------------------- config.h
def le_config():
    txt = open(os.path.join(RAIZ, "firmware", "sumo_esp32", "config.h"), encoding="utf-8").read()
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
        borda.append({"nome": nome, "pino": int(pino), "x": float(x), "y": float(y), "frente": frente == "true"})
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
    txt = open(os.path.join(DADOS, "pecas.js"), encoding="utf-8").read()
    return txt, json.loads(txt[txt.index("{"):txt.rindex("}") + 1])


def malha(dados):
    import numpy as np
    p = np.frombuffer(base64.b64decode(dados["p"]), dtype=np.int16).reshape(-1, 3) * 0.01
    i = np.frombuffer(base64.b64decode(dados["i"]), dtype=np.uint32 if dados["i32"] else np.uint16).reshape(-1, 3)
    return p.astype(float), i.astype(np.int64)


# ---------------------------------------------------------------- peças
def le_poses():
    arq = os.path.join(DADOS, "editor.json")
    return json.load(open(arq, encoding="utf-8"))["poses"] if os.path.exists(arq) else {}


def monta_pecas(cfg, conf, poses):
    """Matriz de cada peça: a pose achada no editor (peças idênticas), ou a identidade do modelo (a primeira cópia
    já vem na posição do editor), ou pos e rot do montagem.json (peças procedurais)."""
    pecas, M = [], {}
    for pc in cfg["pecas"]:
        if pc["id"] in poses:
            ps = poses[pc["id"]]
            W = matriz(ps["R"], ps["pos"])
        elif "editor" in cfg["modelos"].get(pc["modelo"], {}):
            W = matriz([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0])
        else:
            W = matriz(rot(pc.get("rot")), pc.get("pos", [0, 0, 0]))
        M[pc["id"]] = W
        pecas.append(dict(pc, m=W))
    tof = {s["nome"]: s for s in conf["tof"]}
    for pc in pecas:
        s = tof.get(pc["id"])
        if s:
            pc["nome"] = f"{pc['nome']} (XSHUT no GPIO {s['xshut']}, endereço {s['end']})"
    for pc in pecas:
        b = next((x for x in conf["borda"] if x["nome"] == pc["id"]), None)
        if b:
            pc["nome"] = f"{pc['nome']} (A0 no GPIO {b['pino']})"
    return pecas, M


def confere_sensores(cfg, conf, M):
    """O config.h e o editor 3D precisam dizer a mesma coisa: lente e ângulo dos ToF, face dos TCRT5000."""
    pin, erros = cfg["pinos"], []
    mod = {p["id"]: p["modelo"] for p in cfg["pecas"]}
    for s in conf["tof"]:
        if s["nome"] not in M:
            continue
        pm = pin[mod[s["nome"]]]
        W = M[s["nome"]]
        lente, feixe = aplica(W, pm["lente"]), gira(W, pm["feixe"])
        ang = math.degrees(math.atan2(feixe[1], feixe[0]))
        da = (ang - s["ang"] + 180) % 360 - 180
        if abs(lente[0] - s["x"]) > 2 or abs(lente[1] - s["y"]) > 2 or abs(da) > 1:
            erros.append(f"{s['nome']}: editor x {lente[0]:.1f}, y {lente[1]:.1f}, ângulo {ang:.1f}; config.h x {s['x']:.0f}, y {s['y']:.0f}, ângulo {s['ang']:.0f}")
    for b in conf["borda"]:
        if b["nome"] not in M:
            continue
        face = aplica(M[b["nome"]], pin["tcrt5000"]["face"])
        if abs(face[0] - b["x"]) > 2 or abs(face[1] - b["y"]) > 2:
            erros.append(f"{b['nome']}: editor x {face[0]:.1f}, y {face[1]:.1f}; config.h x {b['x']:.0f}, y {b['y']:.0f}")
    if erros:
        raise SystemExit("o editor 3D e o config.h discordam:\n  " + "\n  ".join(erros))


# ---------------------------------------------------------------- pinos
PASSO = 2.54
LINHAS = "abcdefghij"
ALCA = 2.0                                    # primeiro trecho do fio, reto, saindo da ponta (mm)
# distância do fim do pino até onde o fio sai do conector, e comprimento do conector (somado ao fio)
SAIDA = {"femea": 8.0, "macho": 9.0, "solda": 2.0, "borne": 6.0}
COMP_CONECTOR = {"femea": 14.0, "macho": 15.0, "solda": 2.0, "borne": 6.0}


class Pinos:
    def __init__(self, cfg, pecas, M):
        self.cfg, self.M = cfg, M
        self.modelo = {p["id"]: p["modelo"] for p in pecas}

    def ponto(self, ref):
        """'peça:pino' -> (posição, direção do conector, tipo do ponto, direção do primeiro trecho do fio, trecho)."""
        pid, nome = ref.split(":", 1)
        W = self.M[pid]
        mod = self.modelo[pid]
        p = self.cfg["pinos"][mod][nome]
        d = gira(W, p["d"])
        h = gira(W, p["h"]) if "h" in p else d
        return aplica(W, p["p"]), d, p["tipo"], h, p.get("alca", ALCA)


def tipo_do_fio(a, b):
    """Jumper pelas duas pontas: fêmea (em pino macho), macho (em furo), solda ou borne."""
    if a in ("solda", "borne") and b in ("solda", "borne"):
        return "S"
    if a in ("solda", "borne") or b in ("solda", "borne"):
        outra = b if a in ("solda", "borne") else a
        return "S" + ("F" if outra == "femea" else "M")
    return "".join(sorted((a[0].upper(), b[0].upper()), reverse=True))


def ponta(tipo_ponto):
    """Ponta do fio que encosta num ponto: fêmea num pino macho, macho num furo, solda ou borne."""
    return {"macho": "femea", "furo": "macho"}.get(tipo_ponto, tipo_ponto)


# ---------------------------------------------------------------- ligações
COR = {"3V3": "#d7372f", "GND": "#1b1b1b", "SDA": "#d81b9a", "SCL": "#ef7d1a", "XSHUT": "#2e9e44",
       "A0": "#1fbcc4", "DIP": "#2f6fd6", "IN1": "#8e44ad", "IN2": "#7b4a1e", "IN3": "#c9a227", "IN4": "#e6e6e6",
       "M+": "#2f9a3e", "M-": "#e3c62b", "BAT+": "#c62828", "BAT-": "#212121", "PONTE": "#f0f0f0", "NIQUEL": "#b9bcc0",
       "START": "#7a5230"}


def ligacoes(conf, cfg):
    L = []

    def liga(de, para, rede, grupo, cor=None, obs=""):
        L.append({"de": de, "para": para, "tipo": None, "rede": rede, "grupo": grupo,
                  "cor": cor or COR.get(rede.split()[0], "#888"), "obs": obs})

    # sensores de oponente: três fios no trio S, V e G do GPIO do XSHUT na extensora; o SDA e o SCL dos sete
    # sensores se juntam em duas emendas soldadas (termorretrátil) atrás da fileira de trás da extensora, e
    # um jumper curto leva cada emenda ao pino S do GPIO 21 ou 22
    for s in conf["tof"]:
        n, g = s["nome"], s["xshut"]
        liga(f"{n}:XSHUT", f"ext:S{g}", f"XSHUT {n}", "Sensores de oponente", COR["XSHUT"], f"GPIO {g}: pino S do trio")
        liga(f"{n}:VIN", f"ext:V{g}", "3V3", "Sensores de oponente", COR["3V3"], f"GPIO {g}: pino V do trio (3,3 V)")
        liga(f"{n}:GND", f"ext:G{g}", "GND", "Sensores de oponente", COR["GND"], f"GPIO {g}: pino G do trio")
        liga(f"{n}:SDA", "emenda_sda:P", "SDA", "I2C", None, "jumper cortado ao meio: fêmea no pino do sensor, ponta soldada na emenda do SDA")
        liga(f"{n}:SCL", "emenda_scl:P", "SCL", "I2C", None, "jumper cortado ao meio: fêmea no pino do sensor, ponta soldada na emenda do SCL")
    liga("emenda_sda:E", f"ext:S{conf['sda']}", "SDA", "I2C", None,
         f"jumper curto da emenda ao GPIO {conf['sda']} (pino S do trio)")
    liga("emenda_scl:E", f"ext:S{conf['scl']}", "SCL", "I2C", None,
         f"jumper curto da emenda ao GPIO {conf['scl']} (pino S do trio)")

    # sensores de borda: VCC, GND e A0 no trio do GPIO; o D0 fica sem fio
    for b in conf["borda"]:
        n, g = b["nome"], b["pino"]
        liga(f"{n}:A0", f"ext:S{g}", f"A0 {n}", "Sensores de borda", COR["A0"], f"GPIO {g} (ADC1): pino S do trio; o D0 fica sem fio")
        liga(f"{n}:VCC", f"ext:V{g}", "3V3", "Sensores de borda", COR["3V3"], f"GPIO {g}: pino V do trio (3,3 V)")
        liga(f"{n}:GND", f"ext:G{g}", "GND", "Sensores de borda", COR["GND"], f"GPIO {g}: pino G do trio")

    # chave DIP (furo A da tampa de trás): as quatro pernas do lado A unidas por fio nu, um fio para o 3V3
    liga("dip:A1", "ext:V" + str(conf["dip"][3]), "3V3", "Chave DIP", COR["3V3"],
         "3,3 V para o comum da chave (as quatro pernas do lado A unidas por fio nu soldado)")
    for k, g in enumerate(conf["dip"]):
        liga(f"dip:B{k + 1}", f"ext:S{g}", f"DIP {k + 1}", "Chave DIP", COR["DIP"], f"GPIO {g} (pull-down interno: via ligada lê 1)")

    # botão de início (furo B da tampa de trás): em paralelo com o BOOT do DevKit (GPIO 0 e GND)
    liga("botao:A", "esp:BOOT1", "START", "Botão de início", COR["START"], "perna do botão BOOT do DevKit (GPIO 0)")
    liga("botao:B", "esp:BOOT2", "START", "Botão de início", COR["START"], "outra perna do botão BOOT (GND); a ordem não importa")

    # ponte H
    liga("ponte:GND", "ext:GGNDf", "GND", "Ponte H", COR["GND"], "terra comum entre a lógica e os motores")
    for k, g in enumerate(conf["in"]):
        liga(f"ponte:IN{k + 1}", f"ext:S{g}", f"IN{k + 1}", "Ponte H", COR[f"IN{k + 1}"], f"GPIO {g}: pino S do trio")
    liga("ponte:OUT1", "motorE:M+", "M+ esquerdo", "Motores", None, "fio 22 AWG soldado no motor e preso no borne")
    liga("ponte:OUT2", "motorE:M-", "M- esquerdo", "Motores", None, "fio 22 AWG soldado no motor e preso no borne")
    liga("ponte:OUT3", "motorD:M+", "M+ direito", "Motores", None, "fio 22 AWG soldado no motor e preso no borne")
    liga("ponte:OUT4", "motorD:M-", "M- direito", "Motores", None, "fio 22 AWG soldado no motor e preso no borne")

    # bateria 3S e BMS (as três células e o BMS ficam por cima da tampa da frente)
    liga("cel1:-", "bms:B-", "BAT- B-", "Bateria 3S", COR["BAT-"])
    liga("cel1:+", "cel2:-", "NIQUEL 1-2", "Bateria 3S", COR["NIQUEL"], "fita de níquel soldada; os polos + das três células ficam do mesmo lado: ou a fita atravessa a célula, ou a célula 2 vai invertida")
    liga("cel2:+", "cel3:-", "NIQUEL 2-3", "Bateria 3S", COR["NIQUEL"], "fita de níquel soldada (mesma observação)")
    liga("cel1:+", "bms:B1", "BAT+ B1", "Bateria 3S", "#e0a01e", "fio de equilíbrio")
    liga("cel2:+", "bms:B2", "BAT+ B2", "Bateria 3S", "#e0a01e", "fio de equilíbrio")
    liga("cel3:+", "bms:B+", "BAT+ B+", "Bateria 3S", COR["BAT+"])
    liga("bms:P+", "ponte:VCC", "BAT+ 12 V", "Bateria 3S", COR["BAT+"],
         "P+ do BMS no borne de alimentação, pela tampa da frente (o esquema não tem chave neste ramo)")
    liga("bms:P-", "ponte:PGND", "BAT- 0 V", "Bateria 3S", COR["BAT-"], "P- do BMS no borne de alimentação, pela tampa da frente")

    # lógica: 18650 e TPS63070 por cima da tampa de trás, chave KCD1 (furo C) e extensora por dentro
    liga("cel_logica:+", "chave_liga:C", "BAT+ lógica", "Lógica", COR["BAT+"], "pelo passa-fios da tampa de trás")
    liga("chave_liga:NA", "tps:VIN", "BAT+ chaveado", "Lógica", COR["BAT+"], "pelo passa-fios da tampa de trás")
    liga("cel_logica:-", "tps:GNDI", "BAT- lógica", "Lógica", COR["BAT-"])
    liga("tps:VOUT", "ext:V3V3", "3V3", "Lógica", COR["3V3"], "saída do TPS63070 em 3,3 V no trilho V da extensora")
    liga("tps:GNDO", "ext:GGND", "GND", "Lógica", COR["GND"], "GND do TPS63070 no trilho G da extensora")
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


def chave_do_fio(f):
    return f"{f['de']}>{f['para']}"


def extremos(f, P):
    """Pontas do fio: onde sai do conector (a0, b0) e o ponto depois do primeiro trecho reto (a1, b1)."""
    pa, da, ta, ha, aa = P.ponto(f["de"])
    pb, db, tb, hb, ab = P.ponto(f["para"])
    pt_a, pt_b = ponta(ta), ponta(tb)
    a0, b0 = soma(pa, da, SAIDA[pt_a]), soma(pb, db, SAIDA[pt_b])
    return {"pa": pa, "da": da, "pb": pb, "db": db, "ponta_de": pt_a, "ponta_para": pt_b,
            "a0": a0, "b0": b0, "a1": soma(a0, ha, aa), "b1": soma(b0, hb, ab)}


def caminhos(L, P, rotas):
    for f in L:
        e = extremos(f, P)
        f["tipo"] = tipo_do_fio(e["ponta_de"], e["ponta_para"])
        f["ponta_de"], f["ponta_para"] = e["ponta_de"], e["ponta_para"]
        meio = rotas.get(chave_do_fio(f), [])                 # sem rota calculada: reta entre as pontas
        pts = [e["a0"], e["a1"]] + [list(map(float, w)) for w in meio] + [e["b1"], e["b0"]]
        curva = catmull(pts, 4)
        comp = sum(dist(curva[k + 1], curva[k]) for k in range(len(curva) - 1))
        f["pa"], f["da"], f["pb"], f["db"] = (arred(e[k], 2) for k in ("pa", "da", "pb", "db"))
        f["pts"] = [arred(v, 1) for v in curva]
        f["mm"] = int(round(comp + COMP_CONECTOR[f["ponta_de"]] + COMP_CONECTOR[f["ponta_para"]]))
        f["rotado"] = bool(meio)


def jumper_padrao(mm):
    for c in (100, 150, 200, 300):
        if mm * 1.15 <= c:
            return c
    return int(math.ceil(mm * 1.15 / 50) * 50)


# ---------------------------------------------------------------- interferências
def interferencias(pecas, malhas, cfg, passo=1.0):
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
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            if grupo[a] in ("chassi", "tampas") and grupo[b] in ("chassi", "tampas"):
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


# ---------------------------------------------------------------- rótulos
def pe_boot(cfg):
    return cfg["pinos"]["esp32"]["BOOT1"]["p"]


def rotulos_gpio(L, P, cfg, M):
    """Rótulos da cena: sensores, GPIO de cada trio da extensora usado, emendas do SDA e do SCL e peças da tampa."""
    pe = cfg["pinos"]["extensora"]
    usados = sorted({r.split(":", 1)[1][1:] for f in L for r in (f["de"], f["para"]) if r.startswith("ext:")})
    out = []
    for pc in cfg["pecas"]:
        pm = cfg["pinos"].get(pc["modelo"], {})
        if "lente" in pm:
            q = aplica(M[pc["id"]], pm["lente"])
            out.append({"txt": pc["id"], "pos": arred([q[0], q[1], q[2] + 9.0], 2), "cor": "#ffffff", "fundo": "rgba(32,105,52,0.9)", "escala": 1.0})
        elif "face" in pm:
            q = aplica(M[pc["id"]], pm["face"])
            out.append({"txt": pc["id"], "pos": arred([q[0], q[1], q[2] + 9.0], 2), "cor": "#ffffff", "fundo": "rgba(28,84,150,0.9)", "escala": 1.0})
    for txt, pos, cor in (("BOOT = START (GPIO 0)", aplica(M["esp"], pe_boot(cfg)), "#ffe9a8"), ("DIP 1 2 3 4", [7.0, 0.0, 51.5], "#cfe0ff"),
                          ("Botão de início", [-5.0, 0.0, 50.5], "#ffe9a8"), ("KCD1 liga e desliga", [-17.0, 0.0, 56.5], "#ffe9a8")):
        out.append({"txt": txt, "pos": arred([pos[0], pos[1], pos[2] + 4.0], 2), "cor": cor})
    for g in usados:
        if "S" + g not in pe:
            continue
        x, y, z = pe["S" + g]["p"]
        txt = ("GPIO " + g) if g.isdigit() else g.rstrip("f")
        out.append({"txt": txt, "pos": arred([x + (-6.5 if x < 40 else 7.0), y, 46.0], 2), "cor": "#cfe0ff"})
    for nome in ("SDA", "SCL"):
        q = aplica(M["emenda_" + nome.lower()], [0, 0, 4])
        out.append({"txt": "emenda " + nome, "pos": arred([q[0], q[1], q[2] + 4.0], 2), "cor": "#ffe9a8"})
    return out


# ---------------------------------------------------------------- página
ESQUELETO = ('<!doctype html>\n<html lang="pt-BR">\n<head>\n<meta charset="utf-8">\n'
             '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n</head>\n<body>\n')


def carrega(com_rotas=True):
    """Tudo o que a página e o roteador de fios usam: configuração, peças com matriz, pinos e ligações."""
    cfg = json.load(open(os.path.join(DADOS, "montagem.json"), encoding="utf-8"))
    conf = le_config()
    pecas, M = monta_pecas(cfg, conf, le_poses())
    confere_sensores(cfg, conf, M)
    P = Pinos(cfg, pecas, M)
    L = ligacoes(conf, cfg)
    arq = os.path.join(DADOS, "rotas_fios.json")
    rj = json.load(open(arq, encoding="utf-8")) if com_rotas and os.path.exists(arq) else {}
    caminhos(L, P, rj.get("rotas", {}))
    cfg["_conectores"] = rj.get("conectores_em_conflito", [])
    return cfg, conf, pecas, M, P, L


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--saida", default=os.path.join(RAIZ, "site", "montagem.html"))
    ap.add_argument("--modelo", default=os.path.join(DADOS, "montagem_src.html"))
    ap.add_argument("--interferencias", action="store_true",
                    help="refaz a conferência de interferências (precisa de trimesh e scipy) e grava doc/montagem/interferencias.json")
    ap.add_argument("--fragmento", action="store_true", help="sem <!doctype>, <head> e <body> (página publicada como artefato)")
    a = ap.parse_args()
    cfg, conf, pecas, M, P, L = carrega()
    pecas_js, pecas_dados = le_pecas_js()
    arq_inter = os.path.join(DADOS, "interferencias.json")
    if a.interferencias:
        malhas = {k: malha(v) for k, v in pecas_dados.items()}
        inter = interferencias(pecas, malhas, cfg)
        with open(arq_inter, "w", encoding="utf-8") as f:
            json.dump(inter, f, ensure_ascii=False, indent=1)
            f.write("\n")
    else:
        inter = json.load(open(arq_inter, encoding="utf-8")) if os.path.exists(arq_inter) else None

    # lista de compras de jumpers (os com solda saem de um jumper cortado)
    compras = {}
    for f in L:
        c = jumper_padrao(f["mm"]) if f["tipo"] not in ("MM", "S") or f["mm"] > 40 else 0
        if f["tipo"] != "S":
            chave = (f["tipo"], c)
            compras[chave] = compras.get(chave, 0) + 1
    ordem = {"FF": 0, "MF": 1, "MM": 2, "SF": 3, "SM": 4}
    lista = [{"tipo": t, "cm": c / 10 if c else None, "n": n} for (t, c), n in sorted(compras.items(), key=lambda kv: (ordem[kv[0][0]], kv[0][1]))]
    dados = {
        "sobre": cfg["sobre"], "provisorio": cfg["provisorio"],
        "modelos": {k: v for k, v in cfg["modelos"].items() if "procedural" in v},
        "pecas": [{"id": p["id"], "modelo": p["modelo"], "nome": p.get("nome", p["id"]), "grupo": p.get("grupo", ""),
                   "cor": p.get("cor", "#888"), "opacidade": p.get("opacidade", 1), "visivel": p.get("visivel", True),
                   "m": [round(p["m"][i][j] + 0.0, 4) + 0.0 for j in range(4) for i in range(4)]} for p in pecas],
        "fios": [{k: f[k] for k in ("de", "para", "tipo", "rede", "grupo", "cor", "obs", "ponta_de", "ponta_para",
                                    "pa", "da", "pb", "db", "pts", "mm", "rotado")} for f in L],
        "rotulos": rotulos_gpio(L, P, cfg, M),
        "compras": lista, "interferencias": inter, "conectores": cfg["_conectores"], "diferencas": cfg["esquematico"]["diferencas"],
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
    n = {t: sum(1 for f in L if f["tipo"] == t) for t in ("FF", "MF", "MM", "SF", "SM", "S")}
    sem = [chave_do_fio(f) for f in L if not f["rotado"]]
    print(f"{a.saida}: {len(html) // 1024} KB, {len(L)} ligações {n}; {len(sem)} sem rota calculada")
    if inter:
        print("interferências:")
        for p in inter:
            print(f"  {p['mm3']:6d} mm³  {p['a']}  x  {p['b']}{'  (encaixe)' if p['encaixe'] else ''}")


if __name__ == "__main__":
    main()
