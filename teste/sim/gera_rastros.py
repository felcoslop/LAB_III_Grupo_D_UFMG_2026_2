"""Rastros de lutas simuladas para o visualizador da página (doc/site/lutas.html).

Roda o simulador (teste/sim/luta, o firmware de verdade compilado com o mundo simulado) uma vez
para cada combinação de estratégia, oponente e cor, sempre com pessoas paradas em volta do dojô,
e grava um JSON por luta mais o indice.json com a lista. O perfil de oponente escuro (DIP 4)
acompanha a cor do oponente, como na luta de verdade. O workflow do GitHub Pages roda este
programa a cada publicação, então os rastros sempre saem do firmware da main.

Uso: python teste/sim/gera_rastros.py BIN PASTA [--sementes 3,7,10] [--tempo S]
"""
import argparse
import concurrent.futures as cf
import json
import os
import pathlib
import subprocess

ESTRATEGIAS = [("rush", 1), ("flank", 2), ("fan_out", 4)]
OPONENTES = ["parado", "rapido", "lento", "errante"]
CORES = ["claro", "escuro"]


def roda(binario, pasta, semente, tempo, estr, dip, oponente, cor):
    if cor == "escuro":
        dip |= 8
    nome = f"{estr}_{oponente}_{cor}_{semente}.json"
    cmd = [binario, "-s", str(semente), "-o", oponente, "-c", cor, "-d", str(dip), "-t", str(tempo),
           "-f", "-r", str(pasta / nome)]
    subprocess.run(cmd, check=True, capture_output=True, timeout=300)
    with open(pasta / nome, encoding="utf-8") as f:
        d = json.load(f)
    return {"arquivo": nome, "estrategia": estr, "oponente": oponente, "cor": cor, "semente": semente,
            "resultado": d["resultado"], "tempo": d["tempo"], "quadros": len(d["quadros"])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("binario")
    ap.add_argument("pasta")
    ap.add_argument("--sementes", default="3,7,10", help="sementes separadas por vírgula (largadas diferentes)")
    ap.add_argument("--tempo", type=float, default=45)
    a = ap.parse_args()
    pasta = pathlib.Path(a.pasta)
    pasta.mkdir(parents=True, exist_ok=True)
    sementes = [int(s) for s in a.sementes.split(",")]
    tarefas = [(a.binario, pasta, s, a.tempo, e, d, o, c)
               for s in sementes for e, d in ESTRATEGIAS for o in OPONENTES for c in CORES]
    with cf.ThreadPoolExecutor(max_workers=os.cpu_count() or 2) as ex:
        lutas = list(ex.map(lambda t: roda(*t), tarefas))
    with open(pasta / "indice.json", "w", encoding="utf-8") as f:
        json.dump({"sementes": sementes, "lutas": lutas}, f, ensure_ascii=False, indent=1)
    vit = sum(1 for l in lutas if l["resultado"] == 1)
    print(f"{len(lutas)} rastros em {pasta} ({vit} vitórias)")


if __name__ == "__main__":
    main()
