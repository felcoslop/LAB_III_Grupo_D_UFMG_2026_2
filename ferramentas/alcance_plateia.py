"""Distância mínima da plateia ao dojô para nenhum ToF a ler, calculada a partir do config.h.

Para cada situação (robô com a ponta da rampa na linha olhando para fora, com a traseira na linha de
costas para a plateia, ou de lado), a plateia é um plano paralelo à borda do robô que encosta na linha,
contado da borda de fora da faixa branca. Um sensor lê o plano quando algum raio do cone (eixo mais ou
menos 12,5 graus) o alcança dentro da leitura máxima do sensor, que é a mesma do firmware
(rastreador_alcance_sensor): o círculo de TOF_ALCANCE_MM em volta do eixo das rodas, limitado a
TOF_ALCANCE_LATERAL_MM nos laterais (80 a 120 graus) e a TOF_ALCANCE_TRAS_MM nos que olham para trás
(120 graus ou mais). Pessoa de verdade, mais estreita que um plano, é lida de menos longe.

Uso: python ferramentas/alcance_plateia.py
"""
import math
import pathlib
import re

RAIZ = pathlib.Path(__file__).resolve().parent.parent
MEIO_CONE = 12.5   # graus (VL53L0X: cone de 25 graus)


def le_config():
    t = (RAIZ / "firmware" / "sumo_esp32" / "config.h").read_text(encoding="utf-8")
    d = {n: float(re.search(r"#define\s+" + n + r"\s+(-?[0-9.]+)", t).group(1))
         for n in ("TOF_ALCANCE_MM", "TOF_ALCANCE_LATERAL_MM", "TOF_ALCANCE_TRAS_MM")}
    # geometria do robô montado: a segunda tabela e o segundo conjunto de medidas do corpo
    ultimo = lambda n: float(re.findall(r"#define\s+" + n + r"\s+(-?[0-9.]+)", t)[-1])
    d["frente"], d["tras"], d["lado"] = ultimo("CORPO_FRENTE_MM"), ultimo("CORPO_TRAS_MM"), ultimo("CORPO_ESQ_MM")
    tabela = t[t.rindex("static const CfgToF TOF[] = {"):]
    tabela = tabela[:tabela.index("};")]
    d["tof"] = [(m.group(1), float(m.group(2)), float(m.group(3)), float(m.group(4)))
                for m in re.finditer(r'\{\s*"(\w+)",\s*\d+,\s*0x[0-9A-Fa-f]+,\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)', tabela)]
    return d


def leitura_max(c, x, y, a):
    ux, uy = math.cos(math.radians(a)), math.sin(math.radians(a))
    su = x * ux + y * uy
    disc = su * su - (x * x + y * y) + c["TOF_ALCANCE_MM"] ** 2
    z = -su + math.sqrt(disc) if disc > 0 else 0
    if abs(a) >= 120:
        return min(z, c["TOF_ALCANCE_TRAS_MM"])
    if abs(a) >= 80:
        return min(z, c["TOF_ALCANCE_LATERAL_MM"])
    return z


def alcance(c, normal_deg, borda):
    """Quanto além da borda do robô (na direção normal_deg) cada sensor consegue ler um plano."""
    nx, ny = math.cos(math.radians(normal_deg)), math.sin(math.radians(normal_deg))
    saida = {}
    for nome, x, y, a in c["tof"]:
        z = leitura_max(c, x, y, a)
        melhor = None
        for da in (-MEIO_CONE, 0, MEIO_CONE):
            cos = math.cos(math.radians(a + da)) * nx + math.sin(math.radians(a + da)) * ny
            if cos > 1e-6:
                p = x * nx + y * ny + z * cos
                melhor = p if melhor is None else max(melhor, p)
        if melhor is not None and melhor - borda > 0:
            saida[nome] = melhor - borda
    return saida


def main():
    c = le_config()
    print("leitura máxima de cada ToF (mm):", {n: round(leitura_max(c, x, y, a)) for n, x, y, a in c["tof"]})
    casos = (("ponta da rampa na linha, olhando para fora", 0, c["frente"]),
             ("traseira na linha, de costas para a plateia", 180, c["tras"]),
             ("de lado na linha (lado esquerdo)", 90, c["lado"]))
    for titulo, normal, borda in casos:
        r = alcance(c, normal, borda)
        pior = max(r, key=r.get) if r else None
        print(f"{titulo}: plateia a mais de {round(r[pior]) if pior else 0} mm da borda de fora da faixa branca"
              + (f" (sensor {pior})" if pior else "") + "; por sensor: " + ", ".join(f"{n} {round(v)}" for n, v in sorted(r.items(), key=lambda kv: -kv[1])))


if __name__ == "__main__":
    main()
