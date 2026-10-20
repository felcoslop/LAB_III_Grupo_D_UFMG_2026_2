#!/usr/bin/env python3
"""Varredura de parâmetros do firmware no simulador de combate (teste/combate).

Compara uma variante do firmware (o Grupo D, fa) com o firmware atual congelado numa cópia
(o adversário, fb), nas mesmas lutas sorteadas, para dizer se uma mudança no config.h ou no
código melhora o placar. Cada luta sorteia a estratégia do Grupo D, a cor e o motor do
adversário, a estratégia dele e o rumo de largada dos dois; as lutas são as mesmas para todas as
variantes da mesma semente inicial, então a comparação é pareada.

Uso:
  python teste/combate/varre.py NOME [--def CHAVE=VALOR ...] [--fw DIR] [--base DIR] [--n 300]
        [--semente0 1000] [--largada frente|lado|esquerda|direita|costas|qualquer] [--motores 319,600,1000]
        [--estrategias rush,flank,fanout] [--contra NOME_ANTERIOR] [--saida DIR]
  NOME        rótulo da variante (o executável e o CSV saem em --saida)
  --def       troca o valor de um #define do config.h na cópia da variante (repetível)
  --fw        diretório do firmware da variante (padrão: cópia de firmware/sumo_esp32 com as --def)
  --base      diretório do firmware do adversário (padrão: firmware/sumo_esp32 do repositório;
              para congelar o firmware atual, copie o diretório e aponte para a cópia)
  --largada   rumo do Grupo D na largada: frente (padrão), lado (os dois), esquerda ou direita (um só), costas ou qualquer
  --contra    CSV de outra variante (pelo NOME) para o placar pareado: lutas em que o resultado melhorou
              e em que piorou
O placar é vitórias, derrotas e empates, e o saldo (vitórias menos derrotas) com o erro padrão.
"""
import argparse
import csv
import math
import os
import random
import re
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))
FW = os.path.join(RAIZ, "firmware", "sumo_esp32")
sys.path.insert(0, AQUI)
from bateria import CAMPOS, roda  # noqa: E402

LARGADAS = {"frente": [0], "lado": [1, 2], "esquerda": [1], "direita": [2], "costas": [3], "qualquer": [0, 1, 2, 3]}


def copia_com_defs(destino, defs):
    """Copia o firmware e troca o valor de cada #define pedido (mantém o comentário da linha)."""
    if os.path.isdir(destino):
        shutil.rmtree(destino)
    shutil.copytree(FW, destino)
    p = os.path.join(destino, "config.h")
    txt = open(p, encoding="utf-8").read()
    for kv in defs:
        k, v = kv.split("=", 1)
        novo, n = re.subn(r"(?m)^(#define\s+%s\s+)(\S+)" % re.escape(k), lambda m: m.group(1) + v, txt)
        if n != 1:
            sys.exit(f"--def {k}: {n} ocorrências de #define no config.h (esperado 1)")
        txt = novo
    open(p, "w", encoding="utf-8").write(txt)


def compila(fw_a, fw_b, saida):
    env = dict(os.environ, FW_A=fw_a, FW_B=fw_b)
    subprocess.run(["bash", os.path.join(AQUI, "compila.sh"), "nativo", saida], cwd=RAIZ, env=env, check=True,
                   stdout=subprocess.DEVNULL)


def sorteia(n, semente0, largada, motores, estrategias):
    r = random.Random(semente0)
    lutas = []
    for i in range(n):
        lutas.append({
            "semente": semente0 + i,
            "estr_a": r.choice(estrategias),
            "cor": r.choice(["preto", "branco"]),
            "motor": r.choice(motores),
            "estr_b": r.choice(["rush", "flank", "fanout"]),
            "pos_a": r.choice(LARGADAS[largada]),
            "pos_b": r.randrange(4),
        })
    return lutas


def luta(binario, d, tempo, mais=()):
    extra = ["-b", d["estr_b"], "-p", str(d["pos_a"]), "-P", str(d["pos_b"])] + list(mais)
    r = roda(binario, d["semente"], d["estr_a"], d["cor"], d["motor"], extra, tempo)
    r["estr_b"] = d["estr_b"]
    return r


def placar(rs):
    v = sum(r["vencedor"] == 0 for r in rs)
    d = sum(r["vencedor"] == 1 for r in rs)
    e = len(rs) - v - d
    n = max(1, len(rs))
    # erro padrão do saldo (v - d) / n para multinomial
    var = (v + d) / n - ((v - d) / n) ** 2
    ep = math.sqrt(max(var, 0) / n)
    return v, d, e, (v - d) / n, ep


def linha(rot, rs):
    v, d, e, s, ep = placar(rs)
    return f"{rot:24s} n={len(rs):4d}  V {v:4d}  D {d:4d}  E {e:4d}  saldo {100*s:+6.1f}% +-{100*ep:4.1f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("nome")
    ap.add_argument("--def", dest="defs", action="append", default=[])
    ap.add_argument("--fw")
    ap.add_argument("--base")
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--semente0", type=int, default=1000)
    ap.add_argument("--largada", default="frente", choices=list(LARGADAS))
    ap.add_argument("--motores", default="319,600,1000")
    ap.add_argument("--estrategias", default="rush,flank,fanout")
    ap.add_argument("--tempo", type=float, default=90)
    ap.add_argument("--pessoas", help="MIN,MAX: distância das pessoas à borda do dojô em metros (padrão do simulador: 0.9,1.3)")
    ap.add_argument("--ruido", help="desvio, em graus, do rumo na largada (padrão do simulador: 1,7)")
    ap.add_argument("--atrito", help="fator no atrito do pneu (0,8 com pó no dojô)")
    ap.add_argument("--extra", default="", help="opções a mais para o executável, por exemplo \"-u 0.8\"")
    ap.add_argument("--rotulo", help="sufixo do nome do CSV (para separar conjuntos com opções diferentes)")
    ap.add_argument("--contra")
    ap.add_argument("--saida", default=os.path.join(RAIZ, "teste", "combate", "varre_saida"))
    ap.add_argument("--sem-compilar", action="store_true", help="usa o executável já compilado em --saida")
    a = ap.parse_args()
    os.makedirs(a.saida, exist_ok=True)
    fw_a = a.fw
    if not fw_a:
        fw_a = os.path.join(a.saida, "fw_" + a.nome)
        copia_com_defs(fw_a, a.defs)
    fw_b = a.base or FW
    exe = os.path.join(a.saida, "combate_" + a.nome)
    if not a.sem_compilar:
        compila(fw_a, fw_b, exe)
    motores = [int(m) for m in a.motores.split(",")]
    lutas = sorteia(a.n, a.semente0, a.largada, motores, a.estrategias.split(","))
    with ThreadPoolExecutor(max_workers=os.cpu_count() or 2) as ex:
        mais = (["-q", a.pessoas] if a.pessoas else []) + (["-n", a.ruido] if a.ruido else []) + (["-u", a.atrito] if a.atrito else []) + a.extra.split()
        rs = list(ex.map(lambda d: luta(exe, d, a.tempo, mais), lutas))
    suf = ("_" + a.rotulo) if a.rotulo else ""
    csv_p = os.path.join(a.saida, f"{a.nome}_{a.largada}_{a.n}_{a.semente0}{suf}.csv")
    with open(csv_p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CAMPOS)
        w.writeheader()
        w.writerows(rs)
    print(linha(a.nome + " " + a.largada, rs))
    for m in motores:
        print("  " + linha(f"motor {m}", [r for r in rs if r["motor_b"] == m]))
    for e in ("rush", "flank", "fanout"):
        sub = [r for r in rs if r["estr_b"] == e]
        if sub:
            print("  " + linha(f"adversário {e}", sub))
    for e in sorted({r["estr_a"] for r in rs}):
        print("  " + linha(f"Grupo D {e}", [r for r in rs if r["estr_a"] == e]))
    print("  " + linha("quedas próprias", [r for r in rs if r["vencedor"] == 1 and r["motivo"] == 2]).split("V")[0] +
          f"{sum(r['vencedor'] == 1 and r['motivo'] == 2 for r in rs)} lutas")
    if a.contra:
        ant = os.path.join(a.saida, f"{a.contra}_{a.largada}_{a.n}_{a.semente0}{suf}.csv")
        ref = list(csv.DictReader(open(ant)))
        pior = melhor = 0
        val = lambda x: int(x["vencedor"])
        pts = lambda x: 1 if val(x) == 0 else -1 if val(x) == 1 else 0
        for x, y in zip(rs, ref):
            dif = pts(x) - pts(y)
            melhor += dif > 0
            pior += dif < 0
        print(f"  pareado contra {a.contra}: melhorou {melhor}, piorou {pior}")


if __name__ == "__main__":
    main()
