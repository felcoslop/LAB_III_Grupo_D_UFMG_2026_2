# gera_guia.py
# Este script gera o guia de montagem dos sensores de oponente a partir das tabelas do config.h,
# então o desenho sempre bate com o código. Uso:
#   python doc/gera_guia.py              guia de 5 sensores (doc/guia_montagem_sensores.pdf)
#   python doc/gera_guia.py --sensores 7 guia de 7 sensores (doc/guia_montagem_sensores_7.pdf)
# As três primeiras páginas têm a mesma estrutura nas duas versões: gabarito 1:1 do robô, mapa de
# ligação e tabela fio a fio com a lista de conferência. A versão de 7 sensores ganha mais duas
# páginas: o gabarito 1:1 da bancada com o par novo (TE e TD) e a pinagem completa do ESP32 com o
# estudo de cobertura que levou às posições.
import argparse, math, os, re
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
arg = argparse.ArgumentParser()
arg.add_argument("--sensores", type=int, choices=(5, 7), default=5)
QTD = arg.parse_args().sensores

FONTES = "/usr/share/fonts/truetype/dejavu"   # no Windows, a pasta das fontes DejaVu instaladas
pdfmetrics.registerFont(TTFont("S", os.path.join(FONTES, "DejaVuSans.ttf")))
pdfmetrics.registerFont(TTFont("SB", os.path.join(FONTES, "DejaVuSans-Bold.ttf")))
pdfmetrics.registerFont(TTFont("M", os.path.join(FONTES, "DejaVuSansMono.ttf")))

# ---- leitura das tabelas do config.h ----
cfg = open(os.path.join(RAIZ, "config.h"), encoding="utf-8").read()
ini = cfg.index("#if GEOMETRIA_BANCADA\n//")                      # primeiro bloco: sensores de oponente
BLOCO_BANCADA = cfg[ini:cfg.index("\n#else\n", ini)]
BLOCO_ROBO = cfg[cfg.index("\n#else\n", ini):cfg.index('#define GEOMETRIA_NOME    "ROBO"')]
LINHA_TOF = r'\{\s*"(\w+)",\s*(\d+),\s*(0x[0-9A-Fa-f]+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)\s*\}'

def tabela(bloco):
    if QTD == 5: bloco = re.sub(r"#if QTD_TOF == 7.*?#endif", "", bloco, flags=re.S)
    return [dict(nome=m[1], xshut=int(m[2]), end=m[3], x=int(m[4]), y=int(m[5]), ang=int(m[6]))
            for m in re.finditer(LINHA_TOF, bloco)]

TOF = tabela(BLOCO_ROBO)
TOF_BANCADA = tabela(BLOCO_BANCADA)
assert len(TOF) == QTD and len(TOF_BANCADA) == QTD, (TOF, TOF_BANCADA)
ib = cfg.index("#if GEOMETRIA_BANCADA", cfg.index("struct CfgBorda"))
BORDA = [dict(nome=m[1], pino=int(m[2]), x=int(m[3]), y=int(m[4]), frente=m[6] == "true")
         for m in re.finditer(r'\{\s*"(\w+)",\s*(\d+),\s*(-?\d+),\s*(-?\d+),\s*([-+]?\d+),\s*(true|false)\s*\}',
                              cfg[ib:cfg.index("#else", ib)])]
num = lambda nome, bloco: float(re.search(nome + r"\s+([\d.]+)", bloco)[1])
F_B, T_B = num("CORPO_FRENTE_MM", BLOCO_BANCADA), num("CORPO_TRAS_MM", BLOCO_BANCADA)
E_B, D_B = num("CORPO_ESQ_MM", BLOCO_BANCADA), num("CORPO_DIR_MM", BLOCO_BANCADA)

COR = {"LE": "#8e5bd6", "FE": "#2f7fd8", "FC": "#1f9e6e", "FD": "#e07b1a", "LD": "#d6457f",
       "TE": "#0e7490", "TD": "#a16207", "DE": "#64748b", "DD": "#64748b"}
DESC = {"LE": "lateral esquerda", "FE": "frente esquerda", "FC": "frente centro", "FD": "frente direita",
        "LD": "lateral direita", "TE": "traseira esquerda", "TD": "traseira direita"}
NOVOS = ("TE", "TD")
ROBO = 152
PCB_L, PCB_E = 25.0, 1.6      # placa do sensor vista de cima (em pé): 25 mm x 1,6 mm
RED, BLU, GRN, YEL = colors.HexColor("#d62828"), colors.HexColor("#1d4ed8"), colors.HexColor("#16a34a"), colors.HexColor("#ca8a04")
CINZA = colors.HexColor("#555555")

saida = os.path.join(AQUI, "guia_montagem_sensores.pdf" if QTD == 5 else "guia_montagem_sensores_7.pdf")
c = canvas.Canvas(saida, pagesize=A4)
c.setTitle(f"Guia de montagem dos sensores ToF ({QTD} sensores)")
c.setAuthor("Grupo D, Laboratório de Sistemas III, UFMG")

def txt(x, y, s, size=9, font="S", color=colors.black, anchor="l"):
    c.setFont(font, size); c.setFillColor(color)
    {"c": c.drawCentredString, "r": c.drawRightString}.get(anchor, c.drawString)(x, y, s)

def escala(bx0, by0):
    c.setStrokeColor(colors.black); c.setLineWidth(1.2)
    c.line(bx0, by0, bx0 + 100 * mm, by0)
    for k in range(0, 101, 10):
        h = 3 * mm if k % 50 == 0 else 1.8 * mm
        c.line(bx0 + k * mm, by0, bx0 + k * mm, by0 + h)
    txt(bx0, by0 - 4 * mm, "0", 7); txt(bx0 + 100 * mm, by0 - 4 * mm, "100 mm", 7, anchor="c")
    txt(bx0 + 108 * mm, by0 - 1 * mm, "a régua precisa marcar exatamente 10,0 cm", 7.5)

def sensor(P, s, cone, rot_off=32, lateral_fora=False, rot=None):
    """desenha um ToF em pé: cone de 25 graus, placa, seta e rótulo"""
    col = colors.HexColor(COR[s["nome"]])
    a = math.radians(s["ang"]); ux, uy = math.cos(a), math.sin(a); px, py = -uy, ux
    lx, ly = P(s["x"], s["y"])
    c.setStrokeColor(col); c.setLineWidth(0.5); c.setDash(1.5, 1.5)
    for da in (-12.5, 12.5):
        b = math.radians(s["ang"] + da)
        c.line(lx, ly, *P(s["x"] + cone * math.cos(b), s["y"] + cone * math.sin(b)))
    c.setDash()
    ca = (s["x"] - ux * PCB_E / 2, s["y"] - uy * PCB_E / 2)
    pts = [(ca[0] + px * PCB_L / 2 + ux * PCB_E / 2, ca[1] + py * PCB_L / 2 + uy * PCB_E / 2),
           (ca[0] - px * PCB_L / 2 + ux * PCB_E / 2, ca[1] - py * PCB_L / 2 + uy * PCB_E / 2),
           (ca[0] - px * PCB_L / 2 - ux * PCB_E / 2, ca[1] - py * PCB_L / 2 - uy * PCB_E / 2),
           (ca[0] + px * PCB_L / 2 - ux * PCB_E / 2, ca[1] + py * PCB_L / 2 - uy * PCB_E / 2)]
    path = c.beginPath(); path.moveTo(*P(*pts[0]))
    for q in pts[1:]: path.lineTo(*P(*q))
    path.close(); c.setFillColor(col); c.setStrokeColor(col); c.setLineWidth(0.5); c.drawPath(path, fill=1, stroke=1)
    c.setLineWidth(1.4)
    tx, ty = P(s["x"] + ux * 20, s["y"] + uy * 20)
    c.line(lx, ly, tx, ty)
    for da in (150, -150):
        b = math.radians(s["ang"] + da)
        c.line(tx, ty, *P(s["x"] + ux * 20 + 4 * math.cos(b), s["y"] + uy * 20 + 4 * math.sin(b)))
    c.setFillColor(colors.white); c.setStrokeColor(colors.black); c.setLineWidth(0.6)
    c.circle(lx, ly, 1.1 * mm, fill=1, stroke=1)
    if rot is not None:                                  # posição do rótulo dada por quem chama
        rx, ry = P(*rot)
    elif abs(s["ang"]) >= 60 and lateral_fora:             # laterais da bancada: rótulo por fora, um pouco atrás
        rx, ry = P(s["x"] - 16, s["y"] + uy * 34)
    elif abs(s["ang"]) >= 60:                            # laterais do robô: rótulo por dentro
        rx, ry = P(s["x"] + 30, s["y"] - uy * 28)
    else:
        rx, ry = P(s["x"] + ux * rot_off, s["y"] + uy * rot_off)
    txt(rx, ry + 1 * mm, f'{s["nome"]}  {s["ang"]:+d}°'.replace("+0°", "0°"), 9.5, "SB", col, "c")
    txt(rx, ry - 2.8 * mm, f'XSHUT no GPIO {s["xshut"]}', 6.8, "S", col, "c")
    txt(rx, ry - 5.6 * mm, f'x={s["x"]} y={s["y"]}', 6.3, "M", colors.HexColor("#444444"), "c")

# ============================ PÁGINA 1: GABARITO 1:1 DO ROBÔ ============================
W, H = A4
cx, cy = W / 2, 150 * mm
def P(xr, yr): return cx - yr * mm, cy + xr * mm

txt(W / 2, H - 16 * mm, f"GABARITO 1:1 · {QTD} SENSORES DE OPONENTE NO ROBÔ (vista de cima)", 13, "SB", anchor="c")
txt(W / 2, H - 22 * mm, "Impressão em TAMANHO REAL (100%, sem \"ajustar à página\"). A barra de 100 mm confere a escala.", 8.5, anchor="c")
c.setStrokeColor(CINZA); c.setLineWidth(0.8); c.setDash(3, 2)
x0, y0 = P(ROBO / 2, ROBO / 2)
c.rect(x0, y0 - ROBO * mm, ROBO * mm, ROBO * mm, stroke=1, fill=0); c.setDash()
txt(cx, y0 - ROBO * mm - 5 * mm, "contorno máximo do robô: 152 × 152 mm (classe de 1 kg)", 8, color=CINZA, anchor="c")
c.setStrokeColor(colors.black); c.setLineWidth(2.5)
c.line(*P(ROBO / 2, ROBO / 2), *P(ROBO / 2, -ROBO / 2))
txt(cx, P(ROBO / 2, 0)[1] + 3 * mm, "FRENTE (lâmina)", 9, "SB", anchor="c")
c.setLineWidth(1.2)
ax, ay = P(40, 0); bx, by = P(-10, 0)
c.line(bx, by, ax, ay); c.line(ax, ay, ax - 3 * mm, ay - 5 * mm); c.line(ax, ay, ax + 3 * mm, ay - 5 * mm)
txt(ax + 3 * mm, ay - 12 * mm, "x (frente)", 7.5)
ex, ey = P(0, 40)
c.setDash(2, 2); c.line(cx, cy, ex, ey); c.setDash()
txt(ex - 1 * mm, ey + 2 * mm, "y (esquerda)", 7.5)
c.setLineWidth(1)
c.circle(cx, cy, 3 * mm); c.line(cx - 6 * mm, cy, cx + 6 * mm, cy); c.line(cx, cy - 6 * mm, cx, cy + 6 * mm)
txt(cx + 4 * mm, cy - 8 * mm, "ORIGEM = centro de giro", 8, "SB")
txt(cx + 4 * mm, cy - 11.5 * mm, "(meio entre as 2 rodas de tração)", 7.5)
c.setStrokeColor(colors.HexColor("#999999")); c.setDash(1, 2)
for sg in (1, -1):
    wx, wy = P(15, sg * 66)
    c.rect(wx - 5 * mm, wy - 30 * mm, 10 * mm, 30 * mm)
c.setDash()
txt(*P(-20, 66), "roda", 7, color=colors.HexColor("#999999"), anchor="c")
txt(*P(-20, -66), "roda", 7, color=colors.HexColor("#999999"), anchor="c")
for s in TOF:
    sensor(P, s, 45 if abs(s["ang"]) < 60 else 24)
escala(20 * mm, 52 * mm)
y = 42 * mm
linhas = [
    ("SB", "Uso do gabarito"),
    ("S", "1. Cada sensor fica EM PÉ sobre o traço colorido, com o chip preto virado para fora, na direção da seta, e os pinos para dentro."),
    ("S", "2. O centro do chip (a lente) fica sobre o ponto branco. A altura boa da lente é de 20 a 25 mm do chão."),
    ("S", "3. No robô, o gabarito colado na base do chassi serve para furar ou posicionar os suportes. Os números x, y e ângulo"),
    ("S", "    são os mesmos da tabela TOF[] do config.h. Com a origem fora do centro do quadrado, x e y de cada lente precisam"),
    ("S", "    de nova medida a partir do meio entre as rodas, e o config.h recebe os valores medidos."),
    ("S", f"4. Cada sensor leva uma etiqueta ({', '.join(s['nome'] for s in TOF)}) e o fio XSHUT da cor indicada: é esse fio que dá o nome ao sensor."),
]
if QTD == 7:
    linhas.append(("S", "5. Com 7 sensores, o config.h precisa de #define QTD_TOF 7 (o padrão é 5)."))
for f, l in linhas:
    txt(18 * mm, y, l, 8 if f == "S" else 9, f); y -= 4.3 * mm
c.showPage()

# ============================ PÁGINA EXTRA (7): GABARITO 1:1 DA BANCADA ============================
if QTD == 7:
    c.setPageSize(A4); W, H = A4
    cx, cy = W / 2, 145 * mm
    def P(xr, yr): return cx - yr * mm, cy + xr * mm
    txt(W / 2, H - 14 * mm, "GABARITO 1:1 DA BANCADA · protoboard com 7 sensores (vista de cima)", 12.5, "SB", anchor="c")
    txt(W / 2, H - 19.5 * mm, "Os 5 sensores de antes continuam no mesmo lugar. TE e TD entram nas quinas de trás, apontando a ±160 graus.", 8, anchor="c")
    x0, y0 = P(F_B, E_B)
    c.setFillColor(colors.HexColor("#f4f4f4")); c.setStrokeColor(colors.black); c.setLineWidth(1)
    c.rect(x0, y0 - (F_B + T_B) * mm, (E_B + D_B) * mm, (F_B + T_B) * mm, fill=1, stroke=1)
    for yy, col in ((E_B - 4, "#d62828"), (E_B - 8, "#1d4ed8"), (-D_B + 8, "#16a34a"), (-D_B + 4, "#ca8a04")):
        c.setStrokeColor(colors.HexColor(col)); c.setLineWidth(0.8); c.line(*P(F_B - 5, yy), *P(-T_B + 5, yy))
    txt(*P(F_B - 10, 0), "FRENTE (linha 63)", 8.5, "SB", anchor="c")
    c.setStrokeColor(colors.black); c.setLineWidth(0.8)
    c.circle(cx, cy, 2.5 * mm); c.line(cx - 5 * mm, cy, cx + 5 * mm, cy); c.line(cx, cy - 5 * mm, cx, cy + 5 * mm)
    txt(cx + 4 * mm, cy - 6 * mm, "origem (centro)", 7.5, "SB")
    for b in BORDA:                                          # sensores de borda, para mostrar a folga
        sg = 1 if b["y"] > 0 else -1; pcb = b["y"] - sg * 11
        x1, y1 = P(b["x"] + 7, pcb + 16); x2, y2 = P(b["x"] - 7, pcb - 16)
        c.setFillColor(colors.HexColor("#cfe0ff")); c.setStrokeColor(colors.HexColor("#1d4fa8")); c.setLineWidth(0.6)
        c.rect(min(x1, x2), min(y1, y2), abs(x2 - x1), abs(y2 - y1), fill=1, stroke=1)
        bx_, by_ = P(b["x"], pcb)
        txt(bx_, by_ - 1 * mm, f'{b["nome"]} (borda)', 6.3, "S", colors.HexColor("#1d4fa8"), "c")
    for s in TOF_BANCADA:
        if s["nome"] in NOVOS:                               # quinas de trás: cone curto e rótulo ao lado, longe da régua
            sg = 1 if s["y"] > 0 else -1
            sensor(P, s, 26, rot=(s["x"] - 8, s["y"] + sg * 46))
        else:
            sensor(P, s, 38, rot_off=30, lateral_fora=True)
        if s["nome"] in NOVOS:                               # destaque do par novo
            c.setStrokeColor(colors.HexColor(COR[s["nome"]])); c.setLineWidth(1.2)
            c.circle(*P(s["x"], s["y"]), 7 * mm, fill=0, stroke=1)
    escala(20 * mm, 32 * mm)
    y = 24 * mm
    for l in ["Azul claro = módulo TCRT5000 dos sensores de borda (desenho só para conferir a folga). Ponto branco = lente do ToF.",
              "TE e TD seguem o mesmo jeito dos outros: placa em pé num palito colado por fora da quina, lente a uns 23 mm da base,",
              "sem tirar os jumpers das linhas 1 a 4. Depois de montar, a posição medida de cada lente substitui a prevista no config.h."]:
        txt(18 * mm, y, l, 7.6); y -= 4 * mm
    c.showPage()

# ============================ MAPA DE LIGAÇÃO ============================
c.setPageSize(landscape(A4)); W, H = landscape(A4)
txt(W / 2, H - 13 * mm, f"MAPA DE LIGAÇÃO · ESP32 (placa de expansão) + protoboard + {QTD} sensores", 13, "SB", anchor="c")
txt(W / 2, H - 19 * mm, "Tudo DESLIGADO durante a ligação dos fios. Energia só pelo USB. A protoboard entra só com as 4 trilhas laterais (linhas 1 a 25).", 8.5, anchor="c")
ex0, ey0, ew, eh = 14 * mm, 30 * mm, 60 * mm, 148 * mm
c.setStrokeColor(colors.black); c.setFillColor(colors.HexColor("#f1f5f9")); c.setLineWidth(1)
c.roundRect(ex0, ey0, ew, eh, 3 * mm, fill=1, stroke=1)
txt(ex0 + ew / 2, ey0 + eh - 7 * mm, "ESP32 na placa de expansão", 9.5, "SB", anchor="c")
txt(ex0 + ew / 2, ey0 + eh - 11.5 * mm, "pino S (sinal) de cada GPIO", 7.5, anchor="c")
txt(ex0 + ew / 2, ey0 + eh - 15 * mm, "NUNCA a fileira V nem 5V/VIN", 7.5, "SB", RED, "c")
esp_pins = [("3V3", RED, "3V3"), ("GND", BLU, "GND (qualquer G)"), ("21", GRN, "GPIO 21 · SDA"), ("22", YEL, "GPIO 22 · SCL")]
esp_pins += [(str(s["xshut"]), colors.HexColor(COR[s["nome"]]), f'GPIO {s["xshut"]} · XSHUT {s["nome"]}') for s in TOF]
ESP = {}
passo_esp = min(11 * mm, (eh - 30 * mm) / len(esp_pins))
for k, (p, col, lab) in enumerate(esp_pins):
    yy = ey0 + eh - 24 * mm - k * passo_esp
    xx = ex0 + ew - 4 * mm
    c.setFillColor(col); c.setStrokeColor(colors.black); c.circle(xx, yy, 1.8 * mm, fill=1, stroke=1)
    txt(ex0 + 4 * mm, yy - 1.2 * mm, lab, 8.5, "SB" if k < 4 else "S")
    ESP[p] = (xx, yy)
bx0, by0, bw, bh = 104 * mm, 24 * mm, 70 * mm, 158 * mm
c.setFillColor(colors.HexColor("#fafafa")); c.setStrokeColor(colors.HexColor("#888888"))
c.roundRect(bx0, by0, bw, bh, 2 * mm, fill=1, stroke=1)
txt(bx0 + bw / 2, by0 + bh + 3 * mm, "PROTOBOARD (vista de cima, linha 1 no alto)", 8.5, "SB", anchor="c")
rails = {"3V3": (bx0 + 6 * mm, RED), "GND": (bx0 + 12 * mm, BLU), "SDA": (bx0 + bw - 12 * mm, GRN), "SCL": (bx0 + bw - 6 * mm, YEL)}
for nome, (rx, col) in rails.items():
    c.setStrokeColor(col); c.setLineWidth(3); c.line(rx, by0 + 6 * mm, rx, by0 + bh - 6 * mm)
    txt(rx, by0 + bh - 4.5 * mm, nome, 6.5, "SB", col, "c")
c.setFillColor(colors.HexColor("#e5e7eb")); c.setStrokeColor(colors.HexColor("#bbbbbb")); c.setLineWidth(0.5)
c.rect(bx0 + 17 * mm, by0 + 6 * mm, bw - 34 * mm, bh - 12 * mm, fill=1, stroke=1)
txt(bx0 + bw / 2, by0 + bh / 2, "livre", 8, "SB", colors.HexColor("#666666"), "c")
txt(bx0 + bw / 2, by0 - 5 * mm, "Fita escrita SDA e SCL nas trilhas da DIREITA.", 7.5, "SB", anchor="c")
def fio(p0, p1, col, w=1.3, dash=None):
    c.setStrokeColor(col); c.setLineWidth(w)
    if dash: c.setDash(*dash)
    path = c.beginPath(); path.moveTo(*p0); mx = (p0[0] + p1[0]) / 2
    path.curveTo(mx, p0[1], mx, p1[1], *p1); c.drawPath(path, stroke=1, fill=0); c.setDash()
def furo(xx, yy, col):
    c.setFillColor(colors.white); c.setStrokeColor(col); c.setLineWidth(1); c.circle(xx, yy, 1.1 * mm, fill=1, stroke=1)
ytop = by0 + bh - 12 * mm
for k, (p, rail) in enumerate([("3V3", "3V3"), ("GND", "GND"), ("21", "SDA"), ("22", "SCL")]):
    rx, col = rails[rail]; yy = ytop - k * 4 * mm
    fio(ESP[p], (rx, yy), col); furo(rx, yy, col)
txt(bx0 - 2 * mm, ytop + 4 * mm, "linhas 1 a 4: fios do ESP32", 6.5, anchor="r")
sx0 = 205 * mm
caixa = min(30 * mm, (bh - 4 * mm) / QTD)          # altura reservada para cada sensor
pino = min(3.6 * mm, (caixa - 5 * mm) / 6)
SEN = {}
for k, s in enumerate(TOF):
    col = colors.HexColor(COR[s["nome"]])
    top = by0 + bh - 2 * mm - k * caixa
    c.setFillColor(colors.HexColor("#faf5ff")); c.setStrokeColor(col); c.setLineWidth(1.2)
    c.roundRect(sx0, top - caixa + 1.5 * mm, 74 * mm, caixa - 2 * mm, 2 * mm, fill=1, stroke=1)
    txt(sx0 + 44 * mm, top - 6 * mm, f'{s["nome"]} · {DESC[s["nome"]]}', 8.3, "SB", col)
    txt(sx0 + 44 * mm, top - 10 * mm, f'endereço {s["end"]}', 7, "M")
    for j, pn in enumerate(["VCC", "GND", "SCL", "SDA", "GPIO1", "XSHUT"]):
        yy = top - 3.2 * mm - j * pino; xx = sx0 + 3 * mm
        c.setFillColor(colors.HexColor("#d4af37")); c.setStrokeColor(colors.black); c.setLineWidth(0.5)
        c.circle(xx, yy, 0.95 * mm, fill=1, stroke=1)
        txt(xx + 2.3 * mm, yy - 0.9 * mm, pn, 6.4, "M", colors.HexColor("#999999") if pn == "GPIO1" else colors.black)
        SEN[(s["nome"], pn)] = (xx, yy)
    txt(sx0 + 15 * mm, SEN[(s["nome"], "GPIO1")][1] - 0.9 * mm, "← não ligar", 6.2, "S", colors.HexColor("#999999"))
    rowy = ytop - 20 * mm - k * ((ytop - 20 * mm - by0 - 12 * mm) / QTD)
    for pn, rail in (("VCC", "3V3"), ("GND", "GND"), ("SDA", "SDA"), ("SCL", "SCL")):
        rx, rc = rails[rail]
        yy = rowy - {"VCC": 0, "GND": 1, "SDA": 2, "SCL": 3}[pn] * 2.6 * mm
        fio(SEN[(s["nome"], pn)], (rx, yy), rc, 0.9); furo(rx, yy, rc)
    fio(ESP[str(s["xshut"])], SEN[(s["nome"], "XSHUT")], col, 1.1, (3, 2))
ly = 15 * mm
txt(14 * mm, ly, "Legenda:", 8, "SB"); xx = 32 * mm
for col, lab in ((RED, "3V3"), (BLU, "GND"), (GRN, "SDA"), (YEL, "SCL")):
    c.setStrokeColor(col); c.setLineWidth(2); c.line(xx, ly + 1 * mm, xx + 8 * mm, ly + 1 * mm); txt(xx + 10 * mm, ly, lab, 8); xx += 24 * mm
c.setStrokeColor(colors.black); c.setLineWidth(1.1); c.setDash(3, 2); c.line(xx, ly + 1 * mm, xx + 8 * mm, ly + 1 * mm); c.setDash()
txt(xx + 10 * mm, ly, "XSHUT: jumper FÊMEA-FÊMEA direto do ESP32 ao sensor (não passa pela protoboard)", 8)
txt(14 * mm, ly - 5.5 * mm, "Linhas cheias: jumper MACHO-FÊMEA (fêmea no pino do sensor ou do ESP32, macho na trilha da protoboard). Um furo por fio.", 8)
c.showPage()

# ============================ TABELA FIO A FIO + LISTA DE CONFERÊNCIA ============================
c.setPageSize(A4); W, H = A4
txt(W / 2, H - 16 * mm, "LIGAÇÃO FIO A FIO · CONFERÊNCIA ANTES DE LIGAR O USB", 13, "SB", anchor="c")
rows = [("Nº", "DE", "JUMPER", "PARA", "")]
rows += [("1", "ESP32 pino 3V3", "macho-fêmea", "trilha + ESQUERDA, linha 1", "3V3"),
         ("2", "ESP32 pino G (GND)", "macho-fêmea", "trilha − ESQUERDA, linha 2", "GND"),
         ("3", "ESP32 GPIO 21 (pino S)", "macho-fêmea", "trilha + DIREITA (SDA), linha 3", "SDA"),
         ("4", "ESP32 GPIO 22 (pino S)", "macho-fêmea", "trilha − DIREITA (SCL), linha 4", "SCL")]
n, linha = 5, 6
for s in TOF:
    nm = s["nome"]
    rows += [(str(n), f"{nm} VCC", "macho-fêmea", f"trilha + ESQUERDA, linha {linha}", "3V3"),
             (str(n + 1), f"{nm} GND", "macho-fêmea", f"trilha − ESQUERDA, linha {linha}", "GND"),
             (str(n + 2), f"{nm} SDA", "macho-fêmea", f"trilha + DIREITA (SDA), linha {linha}", "SDA"),
             (str(n + 3), f"{nm} SCL", "macho-fêmea", f"trilha − DIREITA (SCL), linha {linha}", "SCL"),
             (str(n + 4), f"{nm} XSHUT", "fêmea-fêmea", f"ESP32 GPIO {s['xshut']} (pino S)", nm),
             ("", f"{nm} GPIO1", "-", "NÃO LIGAR", "")]
    n += 5; linha += 3
colx = [14, 24, 70, 100, 165]
alt = 4.25 * mm if QTD == 5 else 3.55 * mm
fonte = 7.6 if QTD == 5 else 7.0
y = H - 26 * mm
CC = {"3V3": RED, "GND": BLU, "SDA": GRN, "SCL": YEL}
for i, r in enumerate(rows):
    f = "SB" if i == 0 else "S"
    if i > 0 and i % 2 == 0:
        c.setFillColor(colors.HexColor("#f3f4f6")); c.rect(12 * mm, y - 1.2 * mm, 186 * mm, alt - 0.1 * mm, fill=1, stroke=0)
    for j, v in enumerate(r[:4]):
        txt(colx[j] * mm, y, v, fonte, f, colors.HexColor("#999999") if r[3] == "NÃO LIGAR" else colors.black)
    if r[4]:
        col = CC.get(r[4], colors.HexColor(COR.get(r[4], "#000000")))
        c.setFillColor(col); c.rect(colx[4] * mm, y - 0.6 * mm, 8 * mm, 2.4 * mm, fill=1, stroke=0)
        txt(colx[4] * mm + 10 * mm, y, r[4], 6.8, "S")
    y -= alt
txt(14 * mm, y - 1 * mm, f"Total: {4 + 4 * QTD} jumpers macho-fêmea e {QTD} fêmea-fêmea. Cada sensor usa 5 dos 6 pinos (o GPIO1 fica livre).", 8, "SB")
y -= 8 * mm
gpios = ", ".join(str(s["xshut"]) for s in TOF)
check = [
    ("SB", "CONFERÊNCIA ANTI-QUEIMA (na ordem)"),
    ("S", "[ ] 1. USB desconectado durante a montagem. Nenhuma fonte na entrada DC da placa de expansão."),
    ("S", "[ ] 2. Energia dos sensores só do pino 3V3 do ESP32. Nunca da fileira V, nem de 5V/VIN (a fileira V pode estar em 5 V)."),
    ("S", "[ ] 3. Trilhas da protoboard: o multímetro no bipe confirma que cada trilha vai da linha 1 à 25 sem corte."),
    ("S", "[ ] 4. Multímetro no bipe, SEM USB: + esquerda com − esquerda NÃO pode bipar (curto na alimentação)."),
    ("S", "[ ] 5. SDA com SCL NÃO pode bipar. SDA com GND e SCL com GND também não."),
    ("S", f"[ ] 6. Cada XSHUT vai só no seu GPIO ({gpios}). Nunca em 3V3, 5V ou GND."),
    ("S", "[ ] 7. Com o USB ligado, o dedo nos sensores e no ESP32 por 10 s: qualquer aquecimento é motivo para desligar na hora."),
    ("S", "[ ] 8. Multímetro em V DC (escala 20 V): trilha + esquerda = 3,3 V. Trilhas SDA e SCL entre 2,5 e 3,3 V."),
    ("SB", "ORDEM DE TESTE"),
    ("S", "1. Só o FC ligado (4 fios + XSHUT). Com o sumo_esp32 gravado, os comandos  lista  e  scan  mostram 0x32 FC."),
    ("S", f"2. Um sensor a mais por vez, sempre com o USB desligado na hora de encaixar. O scan final mostra 0x30 a 0x{0x30 + QTD - 1:02X}."),
    ("S", "3. Um endereço 0x29 no scan indica XSHUT solto ou no GPIO errado."),
    ("S", "4. No modo  id , a mão cobrindo um sensor por vez faz o terminal escrever o nome dele, que precisa bater com a etiqueta."),
    ("S", "5. No modo  ver , uma caixa movida em volta muda o ângulo e o estado. Pronto para a máquina de estados."),
]
if QTD == 7:
    check.insert(9, ("S", "[ ] 9. config.h com #define QTD_TOF 7. Sem isso o firmware usa só os 5 sensores antigos."))
for f, l in check:
    if f == "SB": y -= 1.5 * mm
    txt(14 * mm, y, l, (8.0 if f == "S" else 9.3) if QTD == 7 else (8.2 if f == "S" else 9.5), f)
    y -= 4.5 * mm if QTD == 7 else 4.8 * mm
c.showPage()

# ============================ PÁGINA EXTRA (7): PINAGEM E COBERTURA ============================
if QTD == 7:
    c.setPageSize(A4); W, H = A4
    txt(W / 2, H - 15 * mm, "PINAGEM COMPLETA DO ESP32 E POSIÇÃO DOS 7 SENSORES", 13, "SB", anchor="c")
    pinos = [
        ("21 / 22", "I2C SDA / SCL", "barramento comum dos 7 ToF (100 kHz; 400 kHz na placa definitiva)"),
        ("13 / 14 / 27 / 26 / 25", "XSHUT LE / FE / FC / FD / LD", "os 5 sensores de antes, sem mudança"),
        ("33", "XSHUT TE", "novo, sem restrição de boot"),
        ("15", "XSHUT TD", "novo; pino de boot que precisa de nível alto, o mesmo do XSHUT ligado"),
        ("34 / 35", "borda BFE / BFD", "ADC1, só entrada"),
        ("36 (VP) / 39 (VN)", "borda BTE / BTD", "ADC1, só entrada"),
        ("32", "botão de start", "INPUT_PULLUP, nível baixo = apertado"),
        ("2", "LED de status", "LED azul da placa"),
        ("18 / 19 / 23", "TB6612 PWMA / AIN1 / AIN2", "motor esquerdo; STBY no 3V3"),
        ("4 / 16 / 17", "TB6612 PWMB / BIN1 / BIN2", "motor direito"),
        ("5", "livre", "último pino livre sem risco (serve para um 8º sensor)"),
        ("12", "evitar", "nível alto no boot muda a tensão da flash e o ESP32 pode não ligar"),
        ("1 / 3", "não usar", "serial da USB (gravação e Monitor Serial)"),
    ]
    y = H - 27 * mm
    for i, (g, f, o) in enumerate([("GPIO", "FUNÇÃO", "OBSERVAÇÃO")] + pinos):
        fnt = "SB" if i == 0 else "S"
        if i > 0 and i % 2 == 0:
            c.setFillColor(colors.HexColor("#f3f4f6")); c.rect(12 * mm, y - 1.4 * mm, 186 * mm, 4.4 * mm, fill=1, stroke=0)
        destaque = colors.HexColor(COR["TE"]) if f.endswith("TE") else colors.HexColor(COR["TD"]) if f.endswith("TD") else colors.black
        txt(14 * mm, y, g, 7.8, "SB" if i == 0 else "M", destaque); txt(56 * mm, y, f, 7.8, fnt, destaque); txt(104 * mm, y, o, 7.6, fnt)
        y -= 4.6 * mm
    txt(14 * mm, y - 1 * mm, "O DevKit de 30 pinos expõe 25 GPIOs: com 7 sensores o projeto usa 21, e o 5 é o último livre sem restrição.", 7.8, "SB")

    # diagramas de cobertura (escala 1:18): hoje, a alternativa descartada e a escolhida
    def alcance(x, y_, a, R=542.0):
        ux, uy = math.cos(math.radians(a)), math.sin(math.radians(a)); su = x * ux + y_ * uy
        d = su * su - (x * x + y_ * y_) + R * R
        return -su + math.sqrt(d) if d > 0 else 0
    def diagrama(cxp, cyp, lista, titulo, sub):
        k = 1 / 18.0
        Q = lambda xr, yr: (cxp - yr * k * mm, cyp + xr * k * mm)
        c.setStrokeColor(colors.HexColor("#bbbbbb")); c.setLineWidth(0.5); c.setDash(2, 2)
        c.circle(cxp, cyp, 542 * k * mm, fill=0, stroke=1); c.setDash()
        c.setStrokeColor(colors.black); c.setLineWidth(0.8)
        x1, y1 = Q(76, 76); c.rect(x1, y1 - 152 * k * mm, 152 * k * mm, 152 * k * mm, fill=0, stroke=1)
        for s in lista:
            col = colors.HexColor(COR[s["nome"]]); L = min(alcance(s["x"], s["y"], s["ang"]), 550)
            path = c.beginPath(); path.moveTo(*Q(s["x"], s["y"]))
            for da in [-12.5 + 2.5 * i for i in range(11)]:
                b = math.radians(s["ang"] + da); path.lineTo(*Q(s["x"] + L * math.cos(b), s["y"] + L * math.sin(b)))
            path.close()
            c.setFillColor(col); c.setFillAlpha(0.28); c.setStrokeColor(col); c.setLineWidth(0.4)
            c.drawPath(path, fill=1, stroke=1); c.setFillAlpha(1)
        txt(cxp, cyp - 542 * k * mm - 4.5 * mm, titulo, 8, "SB", anchor="c")
        txt(cxp, cyp - 542 * k * mm - 8.3 * mm, sub, 7, "S", CINZA, "c")
    t5 = [s for s in TOF if s["nome"] not in NOVOS]
    frente = t5 + [dict(nome="DE", x=60, y=62, ang=45), dict(nome="DD", x=60, y=-62, ang=-45)]
    yd = y - 40 * mm
    diagrama(40 * mm, yd, t5, "5 sensores (hoje)", "atrás, de 115° a 180°, não enxerga nada")
    diagrama(105 * mm, yd, frente, "7, par na frente (±45°)", "alternativa descartada")
    diagrama(170 * mm, yd, TOF, "7, par atrás (±160°)", "posição escolhida")
    txt(W / 2, yd + 542 / 18 * mm + 4 * mm, "Cones até o alcance útil de cada sensor, escala 1:18 (círculo cinza: 54,2 cm do centro, o pior caso dentro do dojô)", 7.5, anchor="c")
    y = yd - 542 / 18 * mm - 15 * mm
    texto = [
        ("SB", "Por que o par novo vai atrás"),
        ("S", "O capítulo 2 do Sumo Robot Blackbook [DEDE, s.d.] pede número ímpar de sensores, com um no centro para decidir o ataque,"),
        ("S", "e prioridade para os cantos, que são o ponto fraco da maioria dos robôs. O livro não trata de sensores atrás: o layout de 7"),
        ("S", "dele põe o par extra na frente, em outro ângulo. A simulação abaixo mostra que, com os 5 sensores de hoje, a frente já"),
        ("S", "enxerga bem: o buraco entre FE e LE só existe a mais de uns 25 cm, e o oponente que vem por ali aparece antes de encostar."),
        ("S", "O que falta é a traseira, por onde 83% dos ataques chegam sem ser vistos. Com o par nas quinas de trás, o total continua"),
        ("S", "ímpar e o FC continua sozinho no centro, então a regra do livro vale do mesmo jeito, e a frente não precisa de mais placas."),
        ("SB", "Resultado na simulação (rastreador do firmware compilado no computador, robô de 152 mm)"),
    ]
    for f, l in texto:
        if f == "SB": y -= 1 * mm
        txt(14 * mm, y, l, 7.8 if f == "S" else 8.6, f); y -= 4.1 * mm
    tab = [("", "5 (hoje)", "7, par na frente", "7, par atrás"),
           ("ataques que chegam sem ser vistos", "36%", "36%", "8%"),
           ("ataques por trás que chegam sem ser vistos", "83%", "82%", "9%"),
           ("tempo médio de busca, robô parado (90% dos casos)", "206 ms (700)", "185 ms (660)", "35 ms (140)"),
           ("erro de ângulo na frente, 90% dos casos", "7,5°", "6,1°", "7,5°"),
           ("posições do dojô dentro de algum cone", "58%", "64%", "88%")]
    for i, (a, b5, bf, bt) in enumerate(tab):
        fnt = "SB" if i == 0 else "S"
        txt(16 * mm, y, a, 7.6, fnt); txt(112 * mm, y, b5, 7.6, fnt); txt(140 * mm, y, bf, 7.6, fnt); txt(170 * mm, y, bt, 7.6, "SB")
        y -= 4.2 * mm
    for l in ["Base: 500 ataques em linha reta a 0,4 a 1,2 m/s com o robô parado, 300 buscas seguindo o rumo do firmware e 240 sequências",
              "de rastreio com o oponente andando. O par na frente melhora um pouco a mira nas diagonais, mas não muda o que passa sem ser visto.",
              "Ainda sobra o buraco entre FE e LE (32° a 78°) a mais de 25 cm e uma faixa estreita bem atrás a mais de 30 cm."]:
        txt(14 * mm, y - 1 * mm, l, 7.3); y -= 3.9 * mm
    txt(14 * mm, y - 6 * mm, "Referência", 8.6, "SB")
    txt(14 * mm, y - 10.5 * mm, "DEDE, Fırat Faris. Sumo Robot Blackbook: robot sumo strategies, tactics to win! Istambul: JSumo, [s.d.]. E-book.", 7.6)
    c.showPage()

c.save()
print("ok", saida, len(TOF))
