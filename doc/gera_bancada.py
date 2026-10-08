# Gera "gabarito_bancada.pdf": montagem dos 5 sensores na protoboard (1:1) + vista lateral e cotas.
# Uso: python doc/gera_bancada.py (lê o config.h da pasta do sketch). O gabarito da bancada com
# 7 sensores fica no guia_montagem_sensores_7.pdf, gerado pelo gera_guia.py.
import math, os, re
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors

pdfmetrics.registerFont(TTFont("S", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
pdfmetrics.registerFont(TTFont("SB", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"))
pdfmetrics.registerFont(TTFont("M", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"))

AQUI = os.path.dirname(os.path.abspath(__file__))
cfg = open(os.path.join(AQUI, "..", "config.h"), encoding="utf-8").read()
bloco = cfg[cfg.index("#if GEOMETRIA_BANCADA\n//"):cfg.index("\n#else\n")]
bloco = re.sub(r"#if QTD_TOF == 7.*?#endif", "", bloco, flags=re.S)       # este gabarito é o de 5 sensores
TOF = [dict(nome=m[1], xshut=int(m[2]), x=int(m[4]), y=int(m[5]), ang=int(m[6]))
       for m in re.finditer(r'\{\s*"(\w+)",\s*(\d+),\s*(0x[0-9A-Fa-f]+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)\s*\}', bloco)]
assert len(TOF) == 5
i2 = cfg.index("#if GEOMETRIA_BANCADA", cfg.index("struct CfgBorda"))
blocoB = cfg[i2:cfg.index("#else", i2)]
BORDA = [dict(nome=m[1], pino=int(m[2]), x=int(m[3]), y=int(m[4]), frente=m[6] == "true")
         for m in re.finditer(r'\{\s*"(\w+)",\s*(\d+),\s*(-?\d+),\s*(-?\d+),\s*([-+]?\d+),\s*(true|false)\s*\}', blocoB)]
assert len(BORDA) == 4, BORDA
HB = float(re.search(r"BORDA_ALTURA_MM\s+([\d.]+)", blocoB)[1])
F = float(re.search(r"CORPO_FRENTE_MM\s+([\d.]+)", bloco)[1]); T_ = float(re.search(r"CORPO_TRAS_MM\s+([\d.]+)", bloco)[1])
E = float(re.search(r"CORPO_ESQ_MM\s+([\d.]+)", bloco)[1]); D = float(re.search(r"CORPO_DIR_MM\s+([\d.]+)", bloco)[1])
HL = float(re.search(r"LENTE_ALTURA_MM\s+([\d.]+)", bloco)[1])
COR = {"LE": "#8e5bd6", "FE": "#2f7fd8", "FC": "#1f9e6e", "FD": "#e07b1a", "LD": "#d6457f"}
PCB_L, PCB_E, CHIP_E, PAL_L, PAL_E = 24.6, 1.6, 1.0, 12.0, 2.0

c = canvas.Canvas(os.path.join(AQUI, "gabarito_bancada.pdf"), pagesize=A4)
c.setTitle("Gabarito da bancada (protoboard + 5 sensores)")
W, H = A4
def txt(x, y, s, size=9, font="S", color=colors.black, anchor="l"):
    c.setFont(font, size); c.setFillColor(color)
    {"c": c.drawCentredString, "r": c.drawRightString}.get(anchor, c.drawString)(x, y, s)

# ============================ PÁGINA 1: GABARITO 1:1 ============================
cx, cy = W / 2, 118 * mm
def P(xr, yr): return cx - yr * mm, cy + xr * mm
def poly(pts, fill, stroke, lw=0.6):
    path = c.beginPath(); path.moveTo(*P(*pts[0]))
    for q in pts[1:]: path.lineTo(*P(*q))
    path.close(); c.setFillColor(fill); c.setStrokeColor(stroke); c.setLineWidth(lw); c.drawPath(path, fill=1, stroke=1)
def ret(cxr, cyr, ux, uy, comp, esp):     # retângulo centrado, "comp" ao longo da perpendicular, "esp" ao longo de u
    px, py = -uy, ux
    return [(cxr + px * comp / 2 + ux * esp / 2, cyr + py * comp / 2 + uy * esp / 2),
            (cxr - px * comp / 2 + ux * esp / 2, cyr - py * comp / 2 + uy * esp / 2),
            (cxr - px * comp / 2 - ux * esp / 2, cyr - py * comp / 2 - uy * esp / 2),
            (cxr + px * comp / 2 - ux * esp / 2, cyr + py * comp / 2 - uy * esp / 2)]

txt(W / 2, H - 14 * mm, "GABARITO 1:1 DA BANCADA · protoboard + 5 sensores (vista de cima)", 12.5, "SB", anchor="c")
txt(W / 2, H - 19.5 * mm, "Imprima em TAMANHO REAL (100%). Confira a barra de 100 mm. Cole a folha num papelão e a protoboard sobre o contorno.", 8, anchor="c")

# protoboard
x0, y0 = P(F, E)
c.setFillColor(colors.HexColor("#f4f4f4")); c.setStrokeColor(colors.black); c.setLineWidth(1)
c.rect(x0, y0 - (F + T_) * mm, (E + D) * mm, (F + T_) * mm, fill=1, stroke=1)
for yy, col in ((E - 4, "#d62828"), (E - 8, "#1d4ed8"), (-D + 8, "#16a34a"), (-D + 4, "#ca8a04")):
    c.setStrokeColor(colors.HexColor(col)); c.setLineWidth(0.8); c.line(*P(F - 5, yy), *P(-T_ + 5, yy))
txt(*P(F - 10, 0), "FRENTE (linha 63)", 8.5, "SB", anchor="c"); txt(*P(-T_ + 6, 0), "linha 1", 7.5, anchor="c")
# eixos e origem
c.setStrokeColor(colors.black); c.setLineWidth(0.8)
c.circle(cx, cy, 2.5 * mm); c.line(cx - 5 * mm, cy, cx + 5 * mm, cy); c.line(cx, cy - 5 * mm, cx, cy + 5 * mm)
txt(cx + 4 * mm, cy - 6 * mm, "origem (centro)", 7.5, "SB")
c.setDash(2, 2); c.line(*P(-T_ - 8, 0), *P(F + 55, 0)); c.setDash()
txt(*P(F + 57, 0), "x (0°)", 7.5, anchor="c")
# cotas da protoboard
txt(*P(-T_ - 5, 0), f"{E + D:.2f} mm", 7.5, "M", anchor="c")
c.saveState(); c.translate(*P(-45, E + 5)); c.rotate(90); txt(0, 0, f"{F + T_:.1f} mm", 7.5, "M", anchor="c"); c.restoreState()

for s in TOF:
    col = colors.HexColor(COR[s["nome"]])
    a = math.radians(s["ang"]); ux, uy = math.cos(a), math.sin(a)
    lx, ly = s["x"], s["y"]
    # cone curto
    c.setStrokeColor(col); c.setLineWidth(0.5); c.setDash(1.5, 1.5)
    Lc = 38
    for da in (-12.5, 12.5):
        b = math.radians(s["ang"] + da); c.line(*P(lx, ly), *P(lx + Lc * math.cos(b), ly + Lc * math.sin(b)))
    c.setDash()
    # palito (atrás da placa), placa, chip
    pal_c = (lx - ux * (CHIP_E + PCB_E + PAL_E / 2), ly - uy * (CHIP_E + PCB_E + PAL_E / 2))
    pcb_c = (lx - ux * (CHIP_E + PCB_E / 2), ly - uy * (CHIP_E + PCB_E / 2))
    poly(ret(*pal_c, ux, uy, PAL_L, PAL_E), colors.HexColor("#e8cf9f"), colors.HexColor("#8a6a3a"))
    poly(ret(*pcb_c, ux, uy, PCB_L, PCB_E), col, col)
    poly(ret(lx - ux * CHIP_E / 2, ly - uy * CHIP_E / 2, ux, uy, 4.4, CHIP_E), colors.black, colors.black, 0.3)
    # seta de direção
    c.setStrokeColor(col); c.setLineWidth(1.3)
    tx, ty = lx + ux * 20, ly + uy * 20
    c.line(*P(lx, ly), *P(tx, ty))
    for da in (150, -150):
        b = math.radians(s["ang"] + da); c.line(*P(tx, ty), *P(tx + 3.5 * math.cos(b), ty + 3.5 * math.sin(b)))
    c.setFillColor(colors.white); c.setStrokeColor(colors.black); c.setLineWidth(0.5)
    c.circle(*P(lx, ly), 0.9 * mm, fill=1, stroke=1)
    # rótulo
    if abs(s["ang"]) >= 60: rx, ry = P(lx + 30, ly + uy * 16)
    elif s["ang"] == 0: rx, ry = P(lx + 30, ly - 14)
    else: rx, ry = P(lx + ux * 30 + 6, ly + uy * 30)
    txt(rx, ry + 1 * mm, f'{s["nome"]}  {s["ang"]:+d}°'.replace("+0°", "0°"), 10, "SB", col, "c")
    txt(rx, ry - 2.6 * mm, f'XSHUT GPIO {s["xshut"]}', 6.8, "S", col, "c")
    txt(rx, ry - 5.4 * mm, f'lente x={s["x"]} y={s["y"]}', 6.3, "M", colors.HexColor("#444444"), "c")

# guia de 20° para FE e FD: linha da borda frontal prolongada
for s in TOF:
    if abs(s["ang"]) in (0, 90): continue
    lado = 1 if s["ang"] > 0 else -1
    c.setStrokeColor(colors.HexColor("#999999")); c.setLineWidth(0.4); c.setDash(1, 1.5)
    c.line(*P(F, lado * E), *P(F, lado * (E + 22))); c.setDash()

# sensores de borda (TCRT5000): placa 14 x 32 mm deitada, sensor para BAIXO na ponta de fora
for b in BORDA:
    sg = 1 if b["y"] > 0 else -1
    pcb_cy = b["y"] - sg * 11
    poly([(b["x"] - 7, pcb_cy - 16), (b["x"] + 7, pcb_cy - 16), (b["x"] + 7, pcb_cy + 16), (b["x"] - 7, pcb_cy + 16)],
         colors.HexColor("#cfe0ff"), colors.HexColor("#1d4fa8"), 0.8)
    poly([(b["x"] - 5.1, b["y"] - 2.9), (b["x"] + 5.1, b["y"] - 2.9), (b["x"] + 5.1, b["y"] + 2.9), (b["x"] - 5.1, b["y"] + 2.9)],
         colors.HexColor("#1b2a6b"), colors.black, 0.4)
    for k in range(4):                                   # 4 pinos na ponta de dentro (VCC GND D0 A0)
        c.setFillColor(colors.HexColor("#b8860b")); c.circle(*P(b["x"] - 3.81 + k * 2.54, pcb_cy - sg * 13.5), 0.55 * mm, fill=1, stroke=0)
    c.setFillColor(colors.white); c.setStrokeColor(colors.black); c.circle(*P(b["x"], b["y"]), 0.8 * mm, fill=1, stroke=1)
    lx, ly = P(b["x"] + (9 if b["frente"] else -9), b["y"] + sg * 20)
    txt(lx, ly + 1 * mm, f'{b["nome"]} · GPIO {b["pino"]}', 8, "SB", colors.HexColor("#1d4fa8"), "c")
    txt(lx, ly - 2.2 * mm, f'luzes x={b["x"]} y={b["y"]}', 6.5, "S", colors.HexColor("#1d4fa8"), "c")

# barra de escala
bx0, by0 = 20 * mm, 22 * mm
c.setStrokeColor(colors.black); c.setLineWidth(1.2); c.line(bx0, by0, bx0 + 100 * mm, by0)
for k in range(0, 101, 10):
    h = 3 * mm if k % 50 == 0 else 1.8 * mm; c.line(bx0 + k * mm, by0, bx0 + k * mm, by0 + h)
txt(bx0, by0 - 4 * mm, "0", 7); txt(bx0 + 100 * mm, by0 - 4 * mm, "100 mm", 7, anchor="c")
txt(bx0 + 106 * mm, by0 - 1 * mm, "meça com régua: 10,0 cm exatos", 7.5)
txt(20 * mm, 12 * mm, "Legenda: bege = palito (12 × 2 mm) · cor = placa ToF (24,6 mm) · preto = chip · ponto branco = LENTE/centro do sensor.", 7.3)
txt(20 * mm, 8 * mm, "Azul claro = TCRT5000 (14 × 32 mm) no JEITO A, sensor virado para o CHÃO. No jeito B o módulo fica paralelo à lateral, sensor no mesmo ponto.", 7.3)
c.showPage()

# ============================ PÁGINA 2: MONTAGEM E COTAS ============================
txt(W / 2, H - 14 * mm, "MONTAGEM DOS SENSORES NA BANCADA", 12.5, "SB", anchor="c")

# vista lateral (escala 2:1)
k = 2.0
ox, oy = 30 * mm, H - 105 * mm
def L(xmm, zmm): return ox + xmm * k * mm, oy + zmm * k * mm
c.setStrokeColor(colors.black); c.setLineWidth(1); c.line(*L(-5, 0), *L(75, 0)); txt(*L(76, -1), "mesa", 7.5)
# protoboard
c.setFillColor(colors.HexColor("#f4f4f4")); c.rect(*L(0, 0), 40 * k * mm, 9.5 * k * mm, fill=1, stroke=1)
txt(*L(20, 3.5), "protoboard (9,5 mm)", 7.5, anchor="c")
# palito
xw = 40
c.setFillColor(colors.HexColor("#e8cf9f")); c.setStrokeColor(colors.HexColor("#8a6a3a"))
c.rect(*L(xw, 0), PAL_E * k * mm, 27 * k * mm, fill=1, stroke=1)
# placa (12 mm de altura, lente a HL)
z0 = HL - 6
c.setFillColor(colors.HexColor("#7b2d8e")); c.setStrokeColor(colors.HexColor("#7b2d8e"))
c.rect(*L(xw + PAL_E, z0), PCB_E * k * mm, 12 * k * mm, fill=1, stroke=0)
c.setFillColor(colors.black); c.rect(*L(xw + PAL_E + PCB_E, HL - 1.2), CHIP_E * k * mm, 2.4 * k * mm, fill=1, stroke=0)
# pinos para trás, acima do palito
c.setStrokeColor(colors.HexColor("#b8860b")); c.setLineWidth(1.5)
c.line(*L(xw + PAL_E + PCB_E, z0 + 10.5), *L(xw - 8, z0 + 10.5))
c.setFillColor(colors.HexColor("#333333")); c.rect(*L(xw - 22, z0 + 9.2), 14 * k * mm, 2.6 * k * mm, fill=1, stroke=0)
txt(*L(xw - 15, z0 + 13.5), "jumper fêmea", 7, anchor="c")
# feixe
c.setStrokeColor(colors.HexColor("#1f9e6e")); c.setLineWidth(0.8); c.setDash(2, 2)
xl = xw + PAL_E + PCB_E + CHIP_E
c.line(*L(xl, HL), *L(xl + 28, HL)); c.line(*L(xl, HL), *L(xl + 28, HL + 28 * math.tan(math.radians(12.5))))
c.line(*L(xl, HL), *L(xl + 28, HL - 28 * math.tan(math.radians(12.5)))); c.setDash()
txt(*L(xl + 29, HL - 1), "feixe (±12,5°)", 7.5)
# cotas
def cota_v(x, z1, z2, t):
    c.setStrokeColor(colors.HexColor("#444444")); c.setLineWidth(0.5)
    c.line(*L(x, z1), *L(x, z2)); c.line(*L(x - 1, z1), *L(x + 1, z1)); c.line(*L(x - 1, z2), *L(x + 1, z2))
    txt(*L(x + 1.5, (z1 + z2) / 2 - 0.8), t, 7.5, "M")
cota_v(xl + 6, 0, HL, f"{HL:.0f} mm (centro do chip)")
cota_v(xw - 26, 0, 27, "")
txt(*L(xw - 36, 13), "palito: 27 mm", 7.5, "M")
txt(*L(xw + 5, z0 + 12.5), "placa 12 mm", 7, "S", colors.HexColor("#7b2d8e"))
txt(ox, oy - 8 * mm, "Vista de lado (escala 2:1) do sensor FC. Os outros são iguais, só mudam de lugar.", 8)

# tabela de cotas para o paquímetro
y = oy - 20 * mm
txt(20 * mm, y, "COTAS PARA O PAQUÍMETRO (lente = centro do chip, na face dele)", 9.5, "SB"); y -= 6 * mm
linhas = [
    ("FC", "palito colado no MEIO da borda da frente (27,35 mm de cada lado). Lente a ~4,6 mm à frente da borda."),
    ("LE", "palito colado na lateral esquerda, com o centro a 82,5 mm de cada ponta. Lente a ~4,6 mm para fora."),
    ("LD", "igual ao LE, na lateral direita."),
    ("FE", "no canto frente-esquerdo: lente 4,5 mm à frente da borda da frente e 4,65 mm para fora da lateral."),
    ("", "a placa fica girada 20° em relação à borda da frente (use o traço do gabarito como guia)."),
    ("FD", "espelho do FE, no canto frente-direito."),
    ("todos", f"centro do chip a {HL:.0f} mm da mesa. Chip virado para FORA. Pinos para TRÁS (dentro), acima do palito."),
]
for n, t in linhas:
    if n: txt(20 * mm, y, n, 8.5, "SB", colors.HexColor(COR.get(n, "#000000")))
    txt(34 * mm, y, t, 8.2); y -= 5.2 * mm

y -= 4 * mm
txt(20 * mm, y, "PASSO A PASSO", 9.5, "SB"); y -= 6 * mm
passos = [
    "1. Meça com o paquímetro: espessura do palito, espessura da placa do sensor e altura do centro do chip na placa.",
    "   Se a espessura palito + placa + chip não der ~4,6 mm, corrija x (FC, FE, FD) e y (LE, FE, FD, LD) no config.h.",
    "2. Corte 5 pedaços de palito com 27 mm. Cole cada sensor no palito com cola quente (pouca), chip para fora,",
    "   com o centro do chip a 25 mm da ponta de baixo do palito e a fileira de pinos ACIMA da ponta de cima.",
    "3. Cole o gabarito (página 1) num papelão. Prenda a protoboard sobre o contorno com fita dupla face.",
    "4. Cole cada palito em pé sobre o traço bege do gabarito (FC, LE e LD também encostados na protoboard).",
    "   Confira: a lente sobre o ponto branco, a placa sobre o traço colorido, o palito a 90° da mesa (esquadro).",
    "5. Passe os jumpers por trás, sobre a protoboard, e prenda os fios com fita para não puxarem os sensores.",
    "6. Grave o sumo_esp32 com GEOMETRIA_BANCADA 1. No painel: todos OK, depois 'cal' de cada sensor.",
]
for t in passos: txt(20 * mm, y, t, 8.2); y -= 5 * mm
c.showPage()

# ============================ PÁGINA 3: SENSORES DE BORDA ============================
txt(W / 2, H - 14 * mm, "SENSORES DE BORDA (4x TCRT5000)", 12.5, "SB", anchor="c")
txt(W / 2, H - 19.5 * mm, "Dohyo RoboCore 1 kg: 77 cm de diâmetro, 2,5 cm de altura, laminado PRETO e faixa BRANCA de 2,5 cm na borda.", 8, anchor="c")
k = 2.5
def L3(ox, oy, xmm, zmm): return ox + xmm * k * mm, oy + zmm * k * mm
def vista(ox, oy, titulo, pinos_baixo):
    c.setStrokeColor(colors.black); c.setLineWidth(1); c.line(*L3(ox, oy, -4, 0), *L3(ox, oy, 60, 0))
    txt(*L3(ox, oy, 61, -1), "chão", 7.5)
    lift = 2.0
    for xx in (2, 22):                                   # palitos sob a protoboard
        c.setFillColor(colors.HexColor("#e8cf9f")); c.rect(*L3(ox, oy, xx, 0), 12 * k * mm, lift * k * mm, fill=1, stroke=0)
    c.setFillColor(colors.HexColor("#f4f4f4")); c.setStrokeColor(colors.black)
    c.rect(*L3(ox, oy, 0, lift), 36 * k * mm, 9.5 * k * mm, fill=1, stroke=1)
    txt(*L3(ox, oy, 18, lift + 4), "protoboard", 7.5, anchor="c")
    topo = lift + 9.5
    if pinos_baixo:
        # módulo espetado: pinos para baixo na coluna a, placa acima (espaçador 2,5 mm), sensor pendurado para fora
        pcb_z = topo + 2.5
        c.setFillColor(colors.HexColor("#1d4fa8")); c.rect(*L3(ox, oy, 30, pcb_z), 32 * k * mm, 1.6 * k * mm, fill=1, stroke=0)
        c.setFillColor(colors.HexColor("#222222")); c.rect(*L3(ox, oy, 30.5, topo), 2.5 * k * mm, 2.5 * k * mm, fill=1, stroke=0)
        c.setStrokeColor(colors.HexColor("#b8860b")); c.setLineWidth(1.2); c.line(*L3(ox, oy, 31.7, pcb_z), *L3(ox, oy, 31.7, topo - 5))
        sz = pcb_z - 7
        c.setFillColor(colors.HexColor("#1b2a6b")); c.rect(*L3(ox, oy, 54, sz), 5.8 * k * mm, 7 * k * mm, fill=1, stroke=0)
        xs = 57
    else:
        # palito colado EM CIMA da protoboard, saindo para fora; módulo colado EMBAIXO da ponta do palito
        # (módulo paralelo à lateral, visto aqui de frente: 14 mm de largura), sensor para baixo, pinos para cima
        c.setFillColor(colors.HexColor("#e8cf9f")); c.rect(*L3(ox, oy, 22, topo), 40 * k * mm, 2 * k * mm, fill=1, stroke=0)
        c.setFillColor(colors.HexColor("#1d4fa8")); c.rect(*L3(ox, oy, 44, topo - 1.6), 14 * k * mm, 1.6 * k * mm, fill=1, stroke=0)
        sz = topo - 1.6 - 7
        c.setFillColor(colors.HexColor("#1b2a6b")); c.rect(*L3(ox, oy, 45.9, sz), 10.2 * k * mm, 7 * k * mm, fill=1, stroke=0)
        c.setStrokeColor(colors.HexColor("#b8860b")); c.setLineWidth(1.2); c.setDash(2, 1.5)
        c.line(*L3(ox, oy, 56, topo), *L3(ox, oy, 56, topo + 8)); c.setDash()
        txt(*L3(ox, oy, 57, topo + 6), "pinos p/ cima (na outra ponta)", 6.5)
        xs = 51
    # cota da altura do sensor
    c.setStrokeColor(colors.HexColor("#c0392b")); c.setLineWidth(0.6)
    c.line(*L3(ox, oy, xs + 6, 0), *L3(ox, oy, xs + 6, sz))
    txt(*L3(ox, oy, xs + 7, sz / 2 - 1), "3 a 8 mm", 8, "SB", colors.HexColor("#c0392b"))
    c.setStrokeColor(colors.HexColor("#c0392b")); c.setDash(1.5, 1.5); c.line(*L3(ox, oy, xs, sz), *L3(ox, oy, xs, 0)); c.setDash()
    txt(ox, oy - 7 * mm, titulo, 8.5, "SB")

vista(22 * mm, H - 78 * mm, "JEITO A · pinos saem para BAIXO (mesmo lado do sensor): espete direto na protoboard", True)
vista(22 * mm, H - 138 * mm, "JEITO B · pinos saem para CIMA ou para o lado: palito em cima da protoboard, módulo embaixo da ponta", False)

y = H - 155 * mm
txt(20 * mm, y, "LIGAÇÃO (cada sensor usa a trinca G / V / S do SEU pino na placa de expansão)", 9.5, "SB"); y -= 6 * mm
tab = [("Sensor", "GPIO", "Posição", "Linhas (jeito A)"),
       ("BFE", "34", "frente esquerda", "58 a 61, coluna a"),
       ("BFD", "35", "frente direita", "58 a 61, coluna j"),
       ("BTE", "36 (VP)", "trás esquerda", "1 a 4, coluna a"),
       ("BTD", "39 (VN)", "trás direita", "1 a 4, coluna j")]
for i, r in enumerate(tab):
    for j, v in enumerate(r):
        txt((20 + [0, 22, 48, 90][j]) * mm, y, v, 8.3, "SB" if i == 0 else "S")
    y -= 4.8 * mm
y -= 2 * mm
linhas = [
    "Módulo VCC -> pino V da trinca (3,3 V com o jumper da placa em 3.3V: MEÇA antes, tem que dar 3,3 V).",
    "Módulo GND -> pino G da trinca.   Módulo A0 -> pino S da trinca.   D0: não ligar (o trimpot azul só mexe no D0).",
    "Jeito A: a placa do módulo fica sobre as trilhas + e -; ligue com macho-fêmea do mesmo número de linha (colunas b a e ou f a i)",
    "até a trinca. Mude os 4 fios do ESP32 das trilhas (linhas 1 a 4) para as linhas 21 a 24, senão batem nos sensores de trás.",
    "Jeito B: palito colado em cima da protoboard saindo ~25 mm para fora; módulo colado EMBAIXO da ponta, paralelo à lateral,",
    "sensor para baixo e no ponto branco do gabarito, pinos para cima. Ligue com fêmea-fêmea direto na trinca.",
    "",
    "ALTURA: o sensor rende melhor de 3 a 8 mm do chão. No jeito A a altura depende de quanto a protoboard sobe:",
    "com 1 palito (2 mm) por baixo dá ~7 mm. Meça com o paquímetro. O comando calborda avisa se o contraste ficar fraco.",
    "SUPERFÍCIE DE TESTE: o ideal é o dohyo de verdade. Sem ele: EVA ou cartolina preta fosca com fita/papel branco de 2,5 cm.",
    "Preto para o olho nem sempre é preto para o infravermelho: calibre de novo (calborda) no dohyo da competição.",
    "Luz do sol atrapalha o TCRT5000: teste em ambiente fechado.",
]
for t in linhas:
    txt(20 * mm, y, t, 8.1); y -= 4.6 * mm
c.showPage()
c.save()
print("ok")
