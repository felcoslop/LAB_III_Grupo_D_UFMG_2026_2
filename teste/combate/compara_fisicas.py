#!/usr/bin/env python3
"""Compara as duas físicas do simulador de combate (à mão e MuJoCo) no empurrão frente a frente.

Roda o executável `empurrao` (teste/combate/empurrao.cpp, compilado com MUJOCO=1) nas duas físicas
contra os motores de 319, 600 e 1000 rpm e mostra, para cada uma: quantos empurrões foram decididos
no tempo, o tempo até a decisão, a vitória do Grupo D (319 rpm) e quantas vezes perdeu o robô que
ficou com o nariz levantado pela rampa do outro (o robô com a maior arfagem).

Uso: python teste/combate/compara_fisicas.py BIN [--lutas N] [--semente S] [--motores 319,600,1000]
         [--exige]
  --exige: falha (código 1) se o MuJoCo não decidir 95% dos empurrões em 3 s, se dois robôs iguais
           (319 contra 319) não ficarem entre 30% e 70% de vitórias ou se o robô levantado não perder
           em 85% dos empurrões decididos
"""
import argparse
import csv
import io
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor


def roda(binario, fisica, rpm, n, semente):
    saida = subprocess.run([binario, "-F", fisica, "-m", str(rpm), "-n", str(n), "-s", str(semente)],
                           capture_output=True, text=True, check=True).stdout
    return list(csv.DictReader(io.StringIO("lut,fisica,rpm_b,desalinho_mm,giro_graus,lado_a,vencedor,tempo,arf_a,arf_b,f_max\n" + saida)))


def resumo(linhas):
    n = len(linhas)
    dec = [l for l in linhas if l["vencedor"] in ("0", "1")]
    a = sum(l["vencedor"] == "0" for l in dec)
    lev = [l for l in dec if abs(float(l["arf_a"]) - float(l["arf_b"])) > 5]
    perde_lev = sum((float(l["arf_a"]) > float(l["arf_b"])) == (l["vencedor"] == "1") for l in lev)
    tempos = [float(l["tempo"]) for l in dec]
    return {"n": n, "decididos": len(dec), "rapidos": sum(float(l["tempo"]) <= 3 for l in dec),
            "vence_a": a, "tempo": sum(tempos) / len(tempos) if tempos else None,
            "levantados": len(lev), "levantado_perde": perde_lev}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("binario")
    ap.add_argument("--lutas", type=int, default=100)
    ap.add_argument("--semente", type=int, default=1)
    ap.add_argument("--motores", default="319,600,1000")
    ap.add_argument("--exige", action="store_true")
    a = ap.parse_args()
    motores = [int(m) for m in a.motores.split(",")]
    tarefas = [(f, m) for f in ("simples", "mujoco") for m in motores]
    with ThreadPoolExecutor(max_workers=2) as ex:
        res = list(ex.map(lambda t: resumo(roda(a.binario, t[0], t[1], a.lutas, a.semente)), tarefas))
    print(f"{'física':8s} {'adversário':>10s} {'decididos':>10s} {'em 3 s':>7s} {'t médio':>8s} {'Grupo D vence':>14s} {'levantado perde':>16s}")
    erros = []
    for (f, m), r in zip(tarefas, res):
        t = f"{r['tempo']:.2f} s" if r["tempo"] is not None else "-"
        lev = f"{r['levantado_perde']}/{r['levantados']}"
        print(f"{f:8s} {str(m) + ' rpm':>10s} {r['decididos']:>6d}/{r['n']:<3d} {r['rapidos']:7d} {t:>8s} "
              f"{r['vence_a']:>8d}/{r['decididos']:<5d} {lev:>16s}")
        if f == "mujoco":
            if r["rapidos"] < 0.95 * r["n"]:
                erros.append(f"MuJoCo decidiu só {r['rapidos']} de {r['n']} empurrões contra {m} rpm em 3 s")
            if r["levantados"] and r["levantado_perde"] < 0.85 * r["levantados"]:
                erros.append(f"o robô levantado perdeu só {r['levantado_perde']} de {r['levantados']} contra {m} rpm")
            if m == 319 and r["decididos"] and not 0.30 <= r["vence_a"] / r["decididos"] <= 0.70:
                erros.append(f"dois robôs iguais: o Grupo D venceu {r['vence_a']} de {r['decididos']}")
    if a.exige and erros:
        for e in erros:
            print("ERRO:", e)
        sys.exit(1)


if __name__ == "__main__":
    main()
