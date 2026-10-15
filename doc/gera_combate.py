#!/usr/bin/env python3
"""Monta a página 3D do combate (doc/site/combate.html) em um arquivo só.

Junta o modelo doc/combate_src.html, as malhas do robô (doc/combate/malhas.js, geradas do STL
pelo doc/combate/exporta_malhas.py), o módulo WebAssembly com os dois firmwares
(teste/combate/compila.sh wasm), o resultado da bateria de lutas (teste/combate/bateria.py --json)
e os textos da página (doc/combate/textos.json).

A página fica no repositório (doc/site/combate.html), com a bateria de doc/combate/bateria.json, e o
CI confere que ela é igual à gerada a partir do código atual.

Uso: python doc/gera_combate.py --wasm teste/combate/combate.wasm [--bateria doc/combate/bateria.json]
         [--saida doc/site/combate.html] [--fragmento]
  --fragmento: sem <!doctype>, <head> e <body> (página publicada como artefato)
"""
import argparse
import base64
import json
import os

AQUI = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modelo", default=os.path.join(AQUI, "combate_src.html"))
    ap.add_argument("--malhas", default=os.path.join(AQUI, "combate", "malhas.js"))
    ap.add_argument("--textos", default=os.path.join(AQUI, "combate", "textos.json"))
    ap.add_argument("--wasm", required=True)
    ap.add_argument("--bateria", default=os.path.join(AQUI, "combate", "bateria.json"))
    ap.add_argument("--fragmento", action="store_true")
    ap.add_argument("--saida", default=os.path.join(AQUI, "site", "combate.html"))
    a = ap.parse_args()

    html = open(a.modelo, encoding="utf-8").read()
    malhas = open(a.malhas, encoding="utf-8").read()
    wasm = base64.b64encode(open(a.wasm, "rb").read()).decode()
    bat = json.load(open(a.bateria, encoding="utf-8"))
    textos = json.load(open(a.textos, encoding="utf-8"))

    resumo = []
    for r in bat["resumo"]:
        resumo.append({
            "estrategia": r["estrategia"], "cor": r["cor"], "motor": r["motor"], "lutas": r["lutas"],
            "vitorias": r["vitorias"], "derrotas": r["derrotas_empurrado"] + r["derrotas_saiu_sozinho"],
            "quedas": r["derrotas_saiu_sozinho"], "empates": r["empates"],
        })
    # números dos textos no formato brasileiro (vírgula decimal)
    ag = bat.get("agregados", {})
    br = lambda v: (f"{v:g}".replace(".", ",") if isinstance(v, float) else str(v))
    v = {k: br(x) for k, x in ag.items() if not isinstance(x, dict)}
    for m, p in ag.get("por_motor", {}).items():
        v[f"v{m}"], v[f"d{m}"], v[f"e{m}"], v[f"n{m}"] = p["v"], p["d"], p["e"], p["n"]
        v[f"tv{m}"] = br(float(p.get("tv", 0)))
    for k in ("rush", "flank", "fanout"):
        p = ag.get("por_estrategia", {}).get(k, {"v": 0, "d": 0, "e": 0, "n": 0})
        for c in "vden":
            v[f"{k}_{c}"] = p[c]
    for k in ("frente", "lado", "costas"):
        p = ag.get("por_largada", {}).get(k, {"v": 0, "d": 0, "e": 0, "n": 0})
        for c in "vden":
            v[f"{k}_{c}"] = p[c]
    q = ag.get("quedas", 0)
    v["quedas_texto"] = "não saiu do dojô sozinho em nenhuma luta" if q == 0 else (
        "saiu do dojô sozinho em 1 luta" if q == 1 else f"saiu do dojô sozinho em {q} lutas")
    v["sementes"] = bat.get("sementes", "?")
    try:
        achados = [a.format(**v) for a in textos.get("achados", [])]
        nota = textos.get("nota", "").format(**v)
    except KeyError as e:
        raise SystemExit(f"textos.json usa {e}, que a bateria não trouxe (rode o bateria.py com --json)")
    dados = {"nota": nota, "resumo": resumo, "achados": achados, "como": textos.get("como", [])}

    for marca in ("/*MALHAS*/", '"/*WASM*/"', "/*BATERIA*/"):
        if html.count(marca) != 1:
            raise SystemExit(f"marca {marca} não aparece uma vez só no modelo")
    html = html.replace("/*MALHAS*/", malhas.strip())
    html = html.replace('"/*WASM*/"', json.dumps(wasm))
    html = html.replace("/*BATERIA*/", json.dumps(dados, ensure_ascii=False))
    if not a.fragmento:
        html = ('<!doctype html>\n<html lang="pt-BR">\n<head>\n<meta charset="utf-8">\n'
                '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n</head>\n<body>\n'
                + html + "\n</body>\n</html>\n")
    os.makedirs(os.path.dirname(os.path.abspath(a.saida)), exist_ok=True)
    with open(a.saida, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"{a.saida}: {len(html) / 1024:.0f} KB")


if __name__ == "__main__":
    main()
