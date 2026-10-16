"""Conferência da página do simulador (site/index.html) contra o config.h.

A página roda em JavaScript as mesmas contas do firmware, com as constantes copiadas no objeto
C do script. Este programa lê os #define do config.h, calcula o valor que cada constante da
página deveria ter e falha quando alguma está diferente, para a página nunca mostrar números
de uma versão antiga do firmware. O CI roda este programa em todo pull request.

Uso: python ferramentas/confere_site.py
"""
import math
import pathlib
import re
import sys

RAIZ = pathlib.Path(__file__).resolve().parent.parent

# constante da página -> expressão com os nomes do config.h
MAPA = {
    "B": "BITOLA_MM",
    "RAIO": "RODA_DIAMETRO_MM / 2",
    "KS": "MOTOR_V_PARTIDA",
    "KA": "MOTOR_KA / 1000",
    "VDISP": "BATERIA_V - PONTE_QUEDA_V",
    "QUEDA": "PONTE_QUEDA_V",
    "TAU": "MOTOR_TAU_MS / 1000",
    "ACEL": "ACEL_MAX_MMS2",
    "K_MIRA": "K_MIRA",
    "ANG_CURVA": "ANG_MIRA_CURVA",
    "GIRO_MIRA_MAX": "GIRO_MIRA_MAX_DPS",
    "GIRO_BUSCA": "GIRO_BUSCA_DPS",
    "VEL_BUSCA": "VEL_BUSCA_MMS",
    "GIRO_BORDA": "GIRO_BORDA_DPS",
    "ANG_GIRO_BORDA": "ANG_GIRO_BORDA",
    "T_GIRO_MAX": "T_GIRO_BORDA_MAX_MS",
    "T_RECUO_EXTRA": "T_RECUO_EXTRA_MS",
    "T_BUSCA_GIRO": "T_BUSCA_GIRO_MS",
    "T_BUSCA_AVANCO": "T_BUSCA_AVANCO_MS",
    "CENTRO": "CENTRO_DEG",
    "ALCANCE": "TOF_ALCANCE_MM",
    "PERDE_MS": "FUSAO_PERDE_MS",
    "R_DOJO": "DOHYO_DIAMETRO_MM / 2",
    "FAIXA": "BORDA_FAIXA_MM",
    "LADO": "OPONENTE_LADO_MM",
}


def le_defines(texto):
    """Dicionário nome -> expressão de todos os #define simples do config.h."""
    defs = {}
    for linha in texto.splitlines():
        m = re.match(r"\s*#define\s+([A-Z_][A-Z0-9_]*)\s+(.+?)\s*(//.*)?$", linha)
        if not m or "(" in m.group(1):
            continue
        expr = m.group(2).strip()
        expr = re.sub(r"(\d+\.?\d*)f\b", r"\1", expr)          # 12.0f -> 12.0
        defs[m.group(1)] = expr
    return defs


def avalia(expr, defs, pilha=()):
    """Valor numérico de uma expressão que usa outros #define."""
    def troca(m):
        nome = m.group(0)
        if nome in pilha:
            raise ValueError("definição circular: " + nome)
        if nome not in defs:
            raise KeyError(nome)
        return "(" + str(avalia(defs[nome], defs, pilha + (nome,))) + ")"
    py = re.sub(r"\b[A-Z_][A-Z0-9_]*\b", troca, expr)
    py = py.replace("(float)", "")
    if not re.fullmatch(r"[0-9.+\-*/() eE]+", py):
        raise ValueError("expressão não numérica: " + expr)
    return eval(py)  # só sobram números e operadores


def le_pagina(texto):
    """Objeto C do script da página (constante -> número) e a linha do kV."""
    m = re.search(r"const C = \{(.*?)\};", texto, re.S)
    if not m:
        raise SystemExit("objeto C não encontrado na página")
    pares = re.findall(r"([A-Z_]+):\s*([0-9.]+(?:\s*\*\s*[0-9.]+)*)", m.group(1))
    valores = {k: eval(v) for k, v in pares}
    kv = re.search(r"C\.KV = \((\d+(?:\.\d+)?) - C\.KS\) / \((\d+(?:\.\d+)?) \*", texto)
    return valores, kv


def main():
    defs = le_defines((RAIZ / "firmware" / "sumo_esp32" / "config.h").read_text(encoding="utf-8"))
    pagina = (RAIZ / "site" / "index.html").read_text(encoding="utf-8")
    valores, kv = le_pagina(pagina)
    erros = []
    for chave, expr in MAPA.items():
        if chave not in valores:
            erros.append(f"{chave}: falta na página")
            continue
        certo = avalia(expr, defs)
        if not math.isclose(valores[chave], certo, rel_tol=1e-6, abs_tol=1e-9):
            erros.append(f"{chave}: página {valores[chave]} e config.h {certo} ({expr})")
    if not kv:
        erros.append("linha do C.KV não encontrada na página")
    else:
        if float(kv.group(1)) != avalia("MOTOR_V_NOMINAL", defs):
            erros.append(f"C.KV usa {kv.group(1)} V e o config.h tem MOTOR_V_NOMINAL {avalia('MOTOR_V_NOMINAL', defs)}")
        if float(kv.group(2)) != avalia("MOTOR_RPM_LIVRE", defs):
            erros.append(f"C.KV usa {kv.group(2)} rpm e o config.h tem MOTOR_RPM_LIVRE {avalia('MOTOR_RPM_LIVRE', defs)}")
    if erros:
        for e in erros:
            print("::error::site/index.html desatualizada: " + e)
        sys.exit(1)
    print(f"página do simulador confere com o config.h ({len(MAPA) + 2} constantes)")
    confere_alcances(defs)


# alcance dos ToF desenhado na página 3D do combate (constante da página -> nome no config.h); sem isso os cones
# saem mais compridos que o alcance real dos sensores laterais e traseiros e o oponente "no feixe" não é visto
ALCANCES_COMBATE = {
    "ALCANCE_MM": "TOF_ALCANCE_MM",
    "ALCANCE_LATERAL_MM": "TOF_ALCANCE_LATERAL_MM",
    "ALCANCE_TRAS_MM": "TOF_ALCANCE_TRAS_MM",
}


def confere_alcances(defs):
    fonte = (RAIZ / "doc" / "combate" / "combate_src.html").read_text(encoding="utf-8")
    erros = []
    for nome, define in ALCANCES_COMBATE.items():
        m = re.search(r"\b" + nome + r" = (\d+)", fonte)
        if not m:
            erros.append(f"{nome}: falta no doc/combate/combate_src.html")
        elif int(m.group(1)) != avalia(define, defs):
            erros.append(f"{nome}: página {m.group(1)} e config.h {avalia(define, defs)} ({define})")
    if erros:
        for e in erros:
            print("::error::doc/combate/combate_src.html desatualizada: " + e)
        sys.exit(1)
    print(f"alcances da página do combate conferem com o config.h ({len(ALCANCES_COMBATE)} constantes)")


if __name__ == "__main__":
    main()
