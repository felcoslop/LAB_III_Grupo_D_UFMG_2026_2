#!/usr/bin/env python3
"""Bateria de lutas do simulador de combate (teste/combate).

Roda o executável para cada combinação de estratégia do Grupo D, cor e motor do adversário e
semente, em paralelo, e mostra o placar de cada combinação: vitórias, derrotas (empurrado ou
saindo sozinho) e empates por tempo. A estratégia do adversário é sorteada em cada luta.

Uso: python teste/combate/bateria.py BIN [--sementes N] [--estrategias rush,flank,fanout]
         [--cores preto,branco] [--motores 1000,319] [--atrito 1] [--restit 0.4] [--extra "-p 0"]
         [--csv ARQ] [--json ARQ] [--exige-quedas-proprias N]
  --atrito: fator no atrito do pneu (0,8 com pó no dojô); --restit: restituição do choque;
  --extra: opções a mais para o executável (por exemplo a largada de frente, -p 0)
"""
import argparse
import csv
import io
import json
import os
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

CAMPOS = ["semente", "estr_a", "estr_b", "cor_b", "motor_b", "vencedor", "motivo", "tempo", "contato",
          "levantado_a", "levantado_b", "aquec_a", "aquec_b", "destravas_a", "destravas_b", "prim_contato", "f_max",
          "largada_a", "largada_b"]


def roda(binario, semente, estr, cor, motor, extra, tempo):
    cmd = [binario, "-s", str(semente), "-a", estr, "-c", cor, "-m", str(motor), "-t", str(tempo)] + extra
    saida = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout.strip().splitlines()[-1]
    d = dict(zip(CAMPOS, next(csv.reader(io.StringIO(saida)))))
    for k in ("semente", "motor_b", "vencedor", "motivo", "destravas_a", "destravas_b", "largada_a", "largada_b"):
        d[k] = int(d[k])
    for k in ("tempo", "contato", "levantado_a", "levantado_b", "aquec_a", "aquec_b", "prim_contato", "f_max"):
        d[k] = float(d[k])
    return d


def placar(lutas):
    n = len(lutas)
    v = sum(1 for d in lutas if d["vencedor"] == 0)
    de = sum(1 for d in lutas if d["vencedor"] == 1 and d["motivo"] == 1)
    ds = sum(1 for d in lutas if d["vencedor"] == 1 and d["motivo"] == 2)
    vs = sum(1 for d in lutas if d["vencedor"] == 0 and d["motivo"] == 2)
    e = sum(1 for d in lutas if d["vencedor"] == -1)
    tv = [d["tempo"] for d in lutas if d["vencedor"] == 0]
    return {"lutas": n, "vitorias": v, "derrotas_empurrado": de, "derrotas_saiu_sozinho": ds,
            "vitorias_adversario_saiu": vs, "empates": e,
            "tempo_medio_vitoria": round(sum(tv) / len(tv), 2) if tv else None,
            "aquec_max_a": round(max(d["aquec_a"] for d in lutas), 1) if lutas else None}


def agregados(lutas):
    """Números usados nos textos da página 3D (doc/combate/textos.json)."""
    dec = [d for d in lutas if d["vencedor"] != -1]
    contato = [d["prim_contato"] for d in lutas if d["prim_contato"] >= 0]
    por_motor = {}
    for m in sorted({d["motor_b"] for d in lutas}):
        ls = [d for d in lutas if d["motor_b"] == m]
        tv = [d["tempo"] for d in ls if d["vencedor"] == 0]
        por_motor[str(m)] = {"v": sum(d["vencedor"] == 0 for d in ls), "d": sum(d["vencedor"] == 1 for d in ls),
                             "e": sum(d["vencedor"] == -1 for d in ls), "n": len(ls),
                             "tv": round(statistics.mean(tv), 1) if tv else 0.0}
    # mesma semente, cor e motor com estratégias diferentes: o resultado mudou?
    grupos = {}
    for d in lutas:
        grupos.setdefault((d["semente"], d["cor_b"], d["motor_b"]), set()).add(d["vencedor"])
    emp = [d for d in lutas if d["vencedor"] == -1]
    med = lambda v: round(statistics.median(v), 2) if v else None
    return {
        "lutas": len(lutas), "vitorias": sum(d["vencedor"] == 0 for d in lutas),
        "derrotas": sum(d["vencedor"] == 1 for d in lutas), "empates": len(emp),
        "quedas": sum(d["vencedor"] == 1 and d["motivo"] == 2 for d in lutas),
        "por_motor": por_motor,
        "prim_contato_mediana": med(contato),
        "decidida_mediana": med([d["tempo"] for d in dec]), "decidida_max": round(max((d["tempo"] for d in dec), default=0), 1),
        "empate_contato_medio": round(statistics.mean(d["contato"] for d in emp), 1) if emp else None,
        "aquec_max": round(max(d["aquec_a"] for d in lutas), 1),
        "estrategia_mudou": sum(1 for v in grupos.values() if len(v) > 1), "grupos": len(grupos),
        "por_largada": por_largada(lutas),
        "por_estrategia": {e: {"v": sum(d["vencedor"] == 0 for d in lutas if d["estr_a"] == e),
                               "d": sum(d["vencedor"] == 1 for d in lutas if d["estr_a"] == e),
                               "e": sum(d["vencedor"] == -1 for d in lutas if d["estr_a"] == e),
                               "n": sum(d["estr_a"] == e for d in lutas)}
                           for e in sorted({d["estr_a"] for d in lutas})},
    }


def por_largada(lutas):
    """Placar pelo rumo do Grupo D na largada (de frente para o adversário, de lado, de costas)."""
    nomes = {0: "frente", 1: "lado", 2: "lado", 3: "costas"}
    r = {}
    for d in lutas:
        if "largada_a" not in d:
            continue
        p = r.setdefault(nomes[d["largada_a"]], {"v": 0, "d": 0, "e": 0, "n": 0})
        p["n"] += 1
        p["v" if d["vencedor"] == 0 else "d" if d["vencedor"] == 1 else "e"] += 1
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("binario")
    ap.add_argument("--sementes", type=int, default=20)
    ap.add_argument("--semente0", type=int, default=1)
    ap.add_argument("--estrategias", default="rush,flank,fanout")
    ap.add_argument("--cores", default="preto,branco")
    ap.add_argument("--motores", default="1000,319")
    ap.add_argument("--atrito", type=float, default=None)
    ap.add_argument("--restit", type=float, default=None)
    ap.add_argument("--extra", default="")
    ap.add_argument("--tempo", type=float, default=90)
    ap.add_argument("--csv")
    ap.add_argument("--json")
    ap.add_argument("--exige-quedas-proprias", type=int, default=None,
                    help="falha (código 1) se o robô do Grupo D sair sozinho em mais lutas que isso")
    a = ap.parse_args()
    tarefas = [(s, e, c, int(m)) for e in a.estrategias.split(",") for c in a.cores.split(",")
               for m in a.motores.split(",") for s in range(a.semente0, a.semente0 + a.sementes)]
    extra = a.extra.split()
    if a.atrito is not None:
        extra += ["-u", str(a.atrito)]
    if a.restit is not None:
        extra += ["-e", str(a.restit)]
    with ThreadPoolExecutor(max_workers=os.cpu_count() or 2) as ex:
        lutas = list(ex.map(lambda t: roda(a.binario, t[0], t[1], t[2], t[3], extra, a.tempo), tarefas))
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CAMPOS)
            w.writeheader()
            w.writerows(lutas)
    grupos = {}
    for d in lutas:
        grupos.setdefault((d["estr_a"], d["cor_b"], d["motor_b"]), []).append(d)
    resumo = []
    print(f"{'estratégia':10s} {'adversário':16s} {'lutas':>5s} {'vence':>5s} {'perde':>5s} {'cai só':>6s} {'empata':>6s} {'t vitória':>9s}")
    for (e, c, m), ls in sorted(grupos.items()):
        p = placar(ls)
        p.update({"estrategia": e, "cor": c, "motor": m})
        resumo.append(p)
        tv = f"{p['tempo_medio_vitoria']:.1f} s" if p["tempo_medio_vitoria"] is not None else "-"
        print(f"{e:10s} {c + ' ' + str(m) + ' rpm':16s} {p['lutas']:5d} {p['vitorias']:5d} {p['derrotas_empurrado']:5d} "
              f"{p['derrotas_saiu_sozinho']:6d} {p['empates']:6d} {tv:>9s}")
    total = placar(lutas)
    print(f"{'total':27s} {total['lutas']:5d} {total['vitorias']:5d} {total['derrotas_empurrado']:5d} "
          f"{total['derrotas_saiu_sozinho']:6d} {total['empates']:6d}")
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"resumo": resumo, "total": total, "sementes": a.sementes, "atrito": a.atrito, "restit": a.restit, "agregados": agregados(lutas)},
                      f, indent=1, ensure_ascii=False)
    if a.exige_quedas_proprias is not None and total["derrotas_saiu_sozinho"] > a.exige_quedas_proprias:
        print(f"ERRO: {total['derrotas_saiu_sozinho']} quedas próprias (limite {a.exige_quedas_proprias})")
        sys.exit(1)


if __name__ == "__main__":
    main()
