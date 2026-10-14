"""Bateria de lutas simuladas com o firmware de verdade (teste/sim/luta).

Roda o simulador para cada combinação de oponente, cor, posição da chave DIP e semente, em
paralelo, e mostra uma tabela por estratégia e oponente: vitórias, derrotas (empurrado para
fora), quedas próprias (saiu do dojô sem contato), empates (90 s), tempo médio até vencer,
tempo de contato e tempo mirando objetos fora do dojô. Com --exige, a bateria falha quando
aparece qualquer queda própria ou quando o robô persegue objetos de fora por mais tempo que o
limite, o que serve de teste automático no CI.

Uso: python teste/sim/roda_lutas.py BIN [--sementes N] [--dips 0,1,2,4] [--oponentes a,b]
                                        [--cores claro,escuro] [--fora] [--perfil-pela-cor]
                                        [--exige] [--csv ARQ] [--json ARQ]
"""
import argparse
import collections
import concurrent.futures as cf
import json
import os
import subprocess
import sys

CAMPOS = ["semente", "oponente", "cor", "dip", "resultado", "tempo", "contato", "bordas", "fantasma",
          "bateria", "jeito_nosso", "jeito_op", "deteccao", "estabilidade"]
NOME_DIP = {0: "padrão", 1: "rush", 2: "flank", 4: "fan out"}


def media(v):
    return sum(v) / len(v) if v else 0.0


def roda(binario, semente, oponente, cor, dip, fora, tempo):
    cmd = [binario, "-s", str(semente), "-o", oponente, "-c", cor, "-d", str(dip), "-t", str(tempo)]
    if fora:
        cmd.append("-f")
    saida = subprocess.run(cmd, capture_output=True, text=True, timeout=300).stdout.strip().splitlines()
    linha = saida[-1] if saida else ""
    valores = linha.split(",")
    if len(valores) != len(CAMPOS):
        raise RuntimeError(f"saída inesperada de {' '.join(cmd)}: {linha!r}")
    d = dict(zip(CAMPOS, valores))
    for k in ("tempo", "contato", "fantasma", "bateria", "deteccao", "estabilidade"):
        d[k] = float(d[k])
    d["dip"] = int(d["dip"])
    d["bordas"] = int(d["bordas"])
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("binario")
    ap.add_argument("--sementes", type=int, default=20)
    ap.add_argument("--semente0", type=int, default=1)
    ap.add_argument("--dips", default="0")
    ap.add_argument("--oponentes", default="parado,pesado,rapido,lento,errante,espelho")
    ap.add_argument("--cores", default="claro,escuro")
    ap.add_argument("--fora", action="store_true", help="pessoas paradas em volta do dojô")
    ap.add_argument("--tempo", type=float, default=90)
    ap.add_argument("--exige", action="store_true", help="falha com queda própria ou perseguição de fantasma")
    ap.add_argument("--fantasma-max", type=float, default=1.0, help="segundos máximos mirando objeto de fora por luta")
    ap.add_argument("--csv", default="")
    ap.add_argument("--json", default="", help="resumo por estratégia, oponente e cor (para a página)")
    ap.add_argument("--perfil-pela-cor", action="store_true",
                    help="liga a DIP 4 (perfil escuro) quando o oponente não é claro, como na luta de verdade")
    ap.add_argument("--processos", type=int, default=os.cpu_count() or 2)
    a = ap.parse_args()

    def dip_de(d, cor):
        return int(d) | (8 if a.perfil_pela_cor and cor != "claro" else 0)
    tarefas = [(a.binario, s, o, c, dip_de(d, c), a.fora, a.tempo)
               for d in a.dips.split(",") for o in a.oponentes.split(",") for c in a.cores.split(",")
               for s in range(a.semente0, a.semente0 + a.sementes)]
    res = []
    with cf.ThreadPoolExecutor(max_workers=a.processos) as ex:
        for r in ex.map(lambda t: roda(*t), tarefas):
            res.append(r)
    if a.csv:
        with open(a.csv, "w", encoding="utf-8") as f:
            f.write(",".join(CAMPOS) + "\n")
            for r in res:
                f.write(",".join(str(r[k]) for k in CAMPOS) + "\n")

    grupos = collections.OrderedDict()
    for r in res:
        grupos.setdefault((r["dip"], r["oponente"], r["cor"]), []).append(r)
    print(f"{'estratégia':10} {'oponente':9} {'cor':7} {'lutas':>5} {'vit':>4} {'der':>4} {'queda':>5} {'emp':>4}"
          f" {'t vit (s)':>9} {'contato':>8} {'bordas':>6} {'fantasma':>8} {'detecção':>8} {'estável':>7}")
    tot = collections.Counter()
    resumo = []
    for (dip, op, cor), lista in grupos.items():
        c = collections.Counter(r["resultado"] for r in lista)
        tv = [r["tempo"] for r in lista if r["resultado"] == "vitoria"]
        resumo.append({"estrategia": NOME_DIP.get(dip & 7, str(dip)), "escuro": bool(dip & 8), "oponente": op, "cor": cor,
                       "lutas": len(lista), "vitorias": c["vitoria"], "derrotas": c["derrota"],
                       "quedas": c["queda_propria"], "empates": c["empate"],
                       "tempo_vitoria": round(sum(tv) / len(tv), 2) if tv else None,
                       "fantasma": round(sum(r["fantasma"] for r in lista) / len(lista), 2)})
        print(f"{NOME_DIP.get(dip & 7, str(dip)) + (' esc' if dip & 8 else ''):10} {op:9} {cor:7} {len(lista):5d}"
              f" {c['vitoria']:4d} {c['derrota']:4d} {c['queda_propria']:5d} {c['empate']:4d}"
              f" {sum(tv) / len(tv) if tv else 0:9.2f} {sum(r['contato'] for r in lista) / len(lista):8.2f}"
              f" {sum(r['bordas'] for r in lista) / len(lista):6.1f} {sum(r['fantasma'] for r in lista) / len(lista):8.2f}"
              f" {media([r['deteccao'] for r in lista if r['deteccao'] >= 0]):8.2f}"
              f" {100 * media([r['estabilidade'] for r in lista if r['estabilidade'] >= 0]):6.0f}%")
        tot.update(c)
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump({"sementes": a.sementes, "fora": a.fora, "grupos": resumo}, f, ensure_ascii=False, indent=1)
    n = len(res)
    print(f"\ntotal: {n} lutas | vitórias {tot['vitoria']} ({100 * tot['vitoria'] / n:.0f}%) | derrotas {tot['derrota']}"
          f" | quedas próprias {tot['queda_propria']} | empates {tot['empate']}")
    if a.exige:
        erros = []
        quedas = [r for r in res if r["resultado"] == "queda_propria"]
        if quedas:
            erros.append(f"{len(quedas)} queda(s) própria(s): " + "; ".join(
                f"semente {r['semente']} {r['oponente']} {r['cor']} dip {r['dip']}" for r in quedas[:10]))
        fant = [r for r in res if r["fantasma"] > a.fantasma_max]
        if fant:
            erros.append(f"{len(fant)} luta(s) mirando objeto fora do dojô por mais de {a.fantasma_max} s")
        for e in erros:
            print("::error::" + e)
        if erros:
            sys.exit(1)


if __name__ == "__main__":
    main()
