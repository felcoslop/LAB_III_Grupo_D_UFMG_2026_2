# Gera "guia_montagem_sensores.pdf": gabarito 1:1 + mapa de ligação + checklist
import math, re
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors

pdfmetrics.registerFont(TTFont("S", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
pdfmetrics.registerFont(TTFont("SB", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"))
pdfmetrics.registerFont(TTFont("M", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"))

# ---- lê a tabela TOF direto do config.h (gabarito sempre igual ao código) ----
cfg = open("/home/claude/sumo_esp32/config.h").read()
cfg = cfg[cfg.index("#else"):cfg.index("#endif", cfg.index("#else"))]   # tabela do ROBÔ
TOF = []
for m in re.finditer(r'\{\s*"(\w+)",\s*(\d+),\s*(0x[0-9A-Fa-f]+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)\s*\}', cfg):
    TOF.append(dict(nome=m[1], xshut=int(m[2]), end=m[3], x=int(m[4]), y=int(m[5]), ang=int(m[6])))
assert len(TOF) == 5, TOF
COR = {"LE": "#8e5bd6", "FE": "#2f7fd8", "FC": "#1f9e6e", "FD": "#e07b1a", "LD": "#d6457f"}
DESC = {"LE": "lateral esquerda", "FE": "frente esquerda", "FC": "frente centro", "FD": "frente direita", "LD": "lateral direita"}
ROBO = 152
PCB_L, PCB_E = 25.0, 1.6      # placa do sensor vista de cima (em pé): 25 mm x 1,6 mm

c = canvas.Canvas("/home/claude/sumo_esp32/doc/guia_montagem_sensores.pdf", pagesize=A4)
c.setTitle("Guia de montagem dos sensores ToF")

def txt(x, y, s, size=9, font="S", color=colors.black, anchor="l"):
    c.setFont(font, size); c.setFillColor(color)
    if anchor == "c": c.drawCentredString(x, y, s)
    elif anchor == "r": c.drawRightString(x, y, s)
    else: c.drawString(x, y, s)

# ============================ PÁGINA 1: GABARITO 1:1 ============================
W, H = A4
cx, cy = W / 2, 150 * mm                    # origem do robô na página
def P(xr, yr):                               # robô (x frente, y esquerda) -> página
    return cx - yr * mm, cy + xr * mm

txt(W / 2, H - 16 * mm, "GABARITO 1:1 · SENSORES DE OPONENTE (vista de cima)", 13, "SB", anchor="c")
txt(W / 2, H - 22 * mm, "Imprima em TAMANHO REAL (100%, desligue \"ajustar à página\"). Confira a barra de 100 mm antes de usar.", 8.5, anchor="c")

# robô
c.setStrokeColor(colors.HexColor("#555555")); c.setLineWidth(0.8); c.setDash(3, 2)
x0, y0 = P(ROBO / 2, ROBO / 2)
c.rect(x0, y0 - ROBO * mm, ROBO * mm, ROBO * mm, stroke=1, fill=0)
c.setDash()
txt(cx, y0 - ROBO * mm - 5 * mm, "contorno máximo do robô: 152 × 152 mm (regra 1 kg)", 8, color=colors.HexColor("#555555"), anchor="c")
# lâmina
c.setStrokeColor(colors.black); c.setLineWidth(2.5)
c.line(*P(ROBO / 2, ROBO / 2), *P(ROBO / 2, -ROBO / 2))
txt(cx, P(ROBO / 2, 0)[1] + 3 * mm, "FRENTE (lâmina)", 9, "SB", anchor="c")
# seta frente
c.setLineWidth(1.2)
ax, ay = P(40, 0); bx, by = P(-10, 0)
c.line(bx, by, ax, ay); c.line(ax, ay, ax - 3 * mm, ay - 5 * mm); c.line(ax, ay, ax + 3 * mm, ay - 5 * mm)
txt(ax + 3 * mm, ay - 12 * mm, "x (frente)", 7.5)
ex, ey = P(0, 40)
c.setDash(2, 2); c.line(cx, cy, ex, ey); c.setDash()
txt(ex - 1 * mm, ey + 2 * mm, "y (esquerda)", 7.5)
# origem
c.setLineWidth(1)
c.circle(cx, cy, 3 * mm); c.line(cx - 6 * mm, cy, cx + 6 * mm, cy); c.line(cx, cy - 6 * mm, cx, cy + 6 * mm)
txt(cx + 4 * mm, cy - 8 * mm, "ORIGEM = centro de giro", 8, "SB")
txt(cx + 4 * mm, cy - 11.5 * mm, "(meio entre as 2 rodas de tração)", 7.5)
# rodas (indicativas)
c.setStrokeColor(colors.HexColor("#999999")); c.setDash(1, 2)
for s in (1, -1):
    wx, wy = P(15, s * 66)
    c.rect(wx - 5 * mm, wy - 30 * mm, 10 * mm, 30 * mm)
c.setDash()
txt(*P(-20, 66), "roda", 7, color=colors.HexColor("#999999"), anchor="c")
txt(*P(-20, -66), "roda", 7, color=colors.HexColor("#999999"), anchor="c")

# sensores
for s in TOF:
    col = colors.HexColor(COR[s["nome"]])
    a = math.radians(s["ang"])
    ux, uy = math.cos(a), math.sin(a)          # direção (robô)
    px, py = -uy, ux                           # perpendicular (robô)
    lx, ly = P(s["x"], s["y"])                 # lente
    # cone 25°
    c.setStrokeColor(col); c.setLineWidth(0.5); c.setDash(1.5, 1.5)
    L = 45 if abs(s["ang"]) < 60 else 24            # laterais: cone curto para caber na folha
    for da in (-12.5, 12.5):
        b = math.radians(s["ang"] + da)
        c.line(lx, ly, *P(s["x"] + L * math.cos(b), s["y"] + L * math.sin(b)))
    c.setDash()
    # placa em pé (traço grosso), com a face do chip no ponto da lente
    ca = (s["x"] - ux * PCB_E / 2, s["y"] - uy * PCB_E / 2)
    pts = [(ca[0] + px * PCB_L / 2 + ux * PCB_E / 2, ca[1] + py * PCB_L / 2 + uy * PCB_E / 2),
           (ca[0] - px * PCB_L / 2 + ux * PCB_E / 2, ca[1] - py * PCB_L / 2 + uy * PCB_E / 2),
           (ca[0] - px * PCB_L / 2 - ux * PCB_E / 2, ca[1] - py * PCB_L / 2 - uy * PCB_E / 2),
           (ca[0] + px * PCB_L / 2 - ux * PCB_E / 2, ca[1] + py * PCB_L / 2 - uy * PCB_E / 2)]
    path = c.beginPath(); path.moveTo(*P(*pts[0]))
    for q in pts[1:]: path.lineTo(*P(*q))
    path.close()
    c.setFillColor(col); c.setStrokeColor(col); c.setLineWidth(0.5)
    c.drawPath(path, fill=1, stroke=1)
    # seta de apontamento
    c.setLineWidth(1.4)
    tx, ty = P(s["x"] + ux * 22, s["y"] + uy * 22)
    c.line(lx, ly, tx, ty)
    for da in (150, -150):
        b = math.radians(s["ang"] + da)
        c.line(tx, ty, *P(s["x"] + ux * 22 + 4 * math.cos(b), s["y"] + uy * 22 + 4 * math.sin(b)))
    c.setFillColor(colors.white); c.setStrokeColor(colors.black); c.setLineWidth(0.6)
    c.circle(lx, ly, 1.2 * mm, fill=1, stroke=1)
    # rótulo
    off = 32
    rx, ry = P(s["x"] + ux * off, s["y"] + uy * off)
    if abs(s["ang"]) >= 60:                          # laterais: rótulo por dentro do robô
        rx, ry = P(s["x"] + 30, s["y"] - uy * 28)
    txt(rx, ry + 1 * mm, f'{s["nome"]}  {s["ang"]:+d}°'.replace("+0°", "0°"), 10, "SB", col, "c")
    txt(rx, ry - 3 * mm, f'XSHUT → GPIO {s["xshut"]}', 7, "S", col, "c")
    txt(rx, ry - 6 * mm, f'x={s["x"]} y={s["y"]}', 6.5, "M", colors.HexColor("#444444"), "c")

# barra de escala
bx0, by0 = 20 * mm, 52 * mm
c.setStrokeColor(colors.black); c.setLineWidth(1.2)
c.line(bx0, by0, bx0 + 100 * mm, by0)
for k in range(0, 101, 10):
    h = 3 * mm if k % 50 == 0 else 1.8 * mm
    c.line(bx0 + k * mm, by0, bx0 + k * mm, by0 + h)
txt(bx0, by0 - 4 * mm, "0", 7); txt(bx0 + 100 * mm, by0 - 4 * mm, "100 mm", 7, anchor="c")
txt(bx0 + 108 * mm, by0 - 1 * mm, "← meça com régua: tem que dar exatamente 10,0 cm", 7.5)

# instruções
y = 42 * mm
linhas = [
    ("SB", "Como usar"),
    ("S", "1. Cada sensor fica EM PÉ sobre o traço colorido: o chip preto virado para fora, na direção da seta; os pinos ficam para dentro do robô."),
    ("S", "2. O centro do chip (a lente) fica sobre o ponto branco. A altura ideal da lente é de 20 a 25 mm do chão."),
    ("S", "3. Na bancada: cole o gabarito na mesa e prenda cada sensor em pé com fita dupla face em um calço (papelão dobrado, borracha)."),
    ("S", "4. No robô: cole o gabarito na base do chassi e use-o para furar ou posicionar os suportes. Os números x, y e ângulo"),
    ("S", "    são os mesmos da tabela TOF[] do config.h. Se a sua origem (meio entre as rodas) não for o centro do quadrado, meça de novo"),
    ("S", "    x e y de cada lente a partir do meio entre as rodas e corrija o config.h. Ângulos: use transferidor a partir da seta x."),
    ("S", "5. Etiquete cada sensor (LE, FE, FC, FD, LD) e use a cor do fio XSHUT indicada: é o fio que dá o nome ao sensor."),
]
for f, l in linhas:
    txt(18 * mm, y, l, 8 if f == "S" else 9, f); y -= 4.3 * mm
c.showPage()

# ============================ PÁGINA 2: MAPA DE LIGAÇÃO ============================
c.setPageSize(landscape(A4)); W, H = landscape(A4)
txt(W / 2, H - 13 * mm, "MAPA DE LIGAÇÃO · ESP32 (placa de expansão) + protoboard + 5 sensores", 13, "SB", anchor="c")
txt(W / 2, H - 19 * mm, "Tudo DESLIGADO enquanto liga os fios. Energia só pelo USB. A protoboard é usada só nas 4 trilhas laterais (linhas 1 a 25).", 8.5, anchor="c")

RED, BLU, GRN, YEL = colors.HexColor("#d62828"), colors.HexColor("#1d4ed8"), colors.HexColor("#16a34a"), colors.HexColor("#ca8a04")

# --- ESP32 + expansão
ex0, ey0, ew, eh = 14 * mm, 38 * mm, 58 * mm, 130 * mm
c.setStrokeColor(colors.black); c.setFillColor(colors.HexColor("#f1f5f9")); c.setLineWidth(1)
c.roundRect(ex0, ey0, ew, eh, 3 * mm, fill=1, stroke=1)
txt(ex0 + ew / 2, ey0 + eh - 7 * mm, "ESP32 na placa de expansão", 9.5, "SB", anchor="c")
txt(ex0 + ew / 2, ey0 + eh - 11.5 * mm, "use o pino S (sinal) de cada GPIO", 7.5, anchor="c")
txt(ex0 + ew / 2, ey0 + eh - 15 * mm, "NUNCA use a fileira V nem 5V/VIN", 7.5, "SB", RED, "c")
esp_pins = [("3V3", RED, "3V3"), ("GND", BLU, "GND (qualquer G)"), ("21", GRN, "GPIO 21 · SDA"), ("22", YEL, "GPIO 22 · SCL"),
            ("13", colors.HexColor(COR["LE"]), "GPIO 13 · XSHUT LE"), ("14", colors.HexColor(COR["FE"]), "GPIO 14 · XSHUT FE"),
            ("27", colors.HexColor(COR["FC"]), "GPIO 27 · XSHUT FC"), ("26", colors.HexColor(COR["FD"]), "GPIO 26 · XSHUT FD"),
            ("25", colors.HexColor(COR["LD"]), "GPIO 25 · XSHUT LD")]
ESP = {}
for k, (p, col, lab) in enumerate(esp_pins):
    yy = ey0 + eh - 26 * mm - k * 11 * mm
    xx = ex0 + ew - 4 * mm
    c.setFillColor(col); c.setStrokeColor(colors.black); c.circle(xx, yy, 1.8 * mm, fill=1, stroke=1)
    txt(ex0 + 4 * mm, yy - 1.2 * mm, lab, 8.5, "SB" if k < 4 else "S")
    ESP[p] = (xx, yy)

# --- protoboard (só as trilhas)
bx0, by0, bw, bh = 104 * mm, 30 * mm, 70 * mm, 150 * mm
c.setFillColor(colors.HexColor("#fafafa")); c.setStrokeColor(colors.HexColor("#888888"))
c.roundRect(bx0, by0, bw, bh, 2 * mm, fill=1, stroke=1)
txt(bx0 + bw / 2, by0 + bh + 3 * mm, "PROTOBOARD (vista de cima, linha 1 no alto)", 8.5, "SB", anchor="c")
rails = {"3V3": (bx0 + 6 * mm, RED, "+  → 3V3"), "GND": (bx0 + 12 * mm, BLU, "−  → GND"),
         "SDA": (bx0 + bw - 12 * mm, GRN, "+  → SDA"), "SCL": (bx0 + bw - 6 * mm, YEL, "−  → SCL")}
for nome, (rx, col, lab) in rails.items():
    c.setStrokeColor(col); c.setLineWidth(3); c.line(rx, by0 + 6 * mm, rx, by0 + bh - 6 * mm)
c.setFillColor(colors.HexColor("#e5e7eb")); c.setStrokeColor(colors.HexColor("#bbbbbb")); c.setLineWidth(0.5)
c.rect(bx0 + 17 * mm, by0 + 6 * mm, bw - 34 * mm, bh - 12 * mm, fill=1, stroke=1)
txt(bx0 + bw / 2, by0 + bh / 2 + 6 * mm, "a b c d e | f g h i j", 7, "M", colors.HexColor("#666666"), "c")
txt(bx0 + bw / 2, by0 + bh / 2, "livre", 8, "SB", colors.HexColor("#666666"), "c")
txt(bx0 + bw / 2, by0 + bh / 2 - 5 * mm, "(depois: borda, capacitores)", 6.5, "S", colors.HexColor("#666666"), "c")
txt(bx0 + 3 * mm, by0 + 2 * mm, "esquerda: + −", 6.5); txt(bx0 + bw - 3 * mm, by0 + 2 * mm, "direita: + −", 6.5, anchor="r")
for nome, (rx, col, lab) in rails.items():
    txt(rx, by0 + bh - 4.5 * mm, nome, 6.5, "SB", col, "c")

txt(bx0 + bw / 2, by0 - 5 * mm, "Cole fita escrita SDA e SCL nas trilhas da DIREITA.", 7.5, "SB", anchor="c")

# linhas ESP -> trilhas (linhas 2 e 3 da protoboard)
def fio(p0, p1, col, w=1.3, dash=None):
    c.setStrokeColor(col); c.setLineWidth(w)
    if dash: c.setDash(*dash)
    path = c.beginPath(); path.moveTo(*p0)
    mx = (p0[0] + p1[0]) / 2
    path.curveTo(mx, p0[1], mx, p1[1], *p1); c.drawPath(path, stroke=1, fill=0)
    c.setDash()
def furo(xx, yy, col):
    c.setFillColor(colors.white); c.setStrokeColor(col); c.setLineWidth(1); c.circle(xx, yy, 1.1 * mm, fill=1, stroke=1)

ytop = by0 + bh - 12 * mm
for k, (p, rail) in enumerate([("3V3", "3V3"), ("GND", "GND"), ("21", "SDA"), ("22", "SCL")]):
    rx, col, _ = rails[rail]
    yy = ytop - k * 4 * mm
    fio(ESP[p], (rx, yy), col); furo(rx, yy, col)
txt(bx0 - 2 * mm, ytop + 4 * mm, "linhas 1-4: fios do ESP32", 6.5, anchor="r")

# sensores
sx0 = 205 * mm
SEN = {}
for k, s in enumerate(TOF):
    col = colors.HexColor(COR[s["nome"]])
    top = by0 + bh - 6 * mm - k * 30 * mm
    c.setFillColor(colors.HexColor("#faf5ff")); c.setStrokeColor(col); c.setLineWidth(1.2)
    c.roundRect(sx0, top - 25 * mm, 72 * mm, 24 * mm, 2 * mm, fill=1, stroke=1)
    txt(sx0 + 46 * mm, top - 7 * mm, f'{s["nome"]} · {DESC[s["nome"]]}', 8.5, "SB", col)
    txt(sx0 + 46 * mm, top - 11.5 * mm, f'endereço {s["end"]}', 7, "M")
    pins = ["VCC", "GND", "SCL", "SDA", "GPIO1", "XSHUT"]
    for j, pn in enumerate(pins):
        yy = top - 3.5 * mm - j * 3.6 * mm
        xx = sx0 + 3 * mm
        c.setFillColor(colors.HexColor("#d4af37")); c.setStrokeColor(colors.black); c.setLineWidth(0.5)
        c.circle(xx, yy, 1.1 * mm, fill=1, stroke=1)
        txt(xx + 2.5 * mm, yy - 1 * mm, pn, 6.8, "M", colors.HexColor("#999999") if pn == "GPIO1" else colors.black)
        SEN[(s["nome"], pn)] = (xx, yy)
    txt(sx0 + 15 * mm, top - 3.5 * mm - 4 * 3.6 * mm - 1 * mm, "← não ligar", 6.5, "S", colors.HexColor("#999999"))
    # fios para as trilhas (linha 6+ da protoboard)
    rowy = ytop - 22 * mm - k * 20 * mm
    for pn, rail in (("VCC", "3V3"), ("GND", "GND"), ("SDA", "SDA"), ("SCL", "SCL")):
        rx, rc, _ = rails[rail]
        yy = rowy - {"VCC": 0, "GND": 1, "SDA": 2, "SCL": 3}[pn] * 3 * mm
        fio(SEN[(s["nome"], pn)], (rx, yy), rc, 0.9); furo(rx, yy, rc)
    # XSHUT direto do ESP (fêmea-fêmea)
    fio(ESP[str(s["xshut"])], SEN[(s["nome"], "XSHUT")], col, 1.1, (3, 2))

# legenda
ly = 20 * mm
txt(14 * mm, ly, "Legenda:", 8, "SB")
items = [(RED, "3V3"), (BLU, "GND"), (GRN, "SDA"), (YEL, "SCL")]
xx = 32 * mm
for col, lab in items:
    c.setStrokeColor(col); c.setLineWidth(2); c.line(xx, ly + 1 * mm, xx + 8 * mm, ly + 1 * mm); txt(xx + 10 * mm, ly, lab, 8); xx += 24 * mm
c.setStrokeColor(colors.black); c.setLineWidth(1.1); c.setDash(3, 2); c.line(xx, ly + 1 * mm, xx + 8 * mm, ly + 1 * mm); c.setDash()
txt(xx + 10 * mm, ly, "XSHUT: jumper FÊMEA-FÊMEA direto do ESP32 ao sensor (não passa pela protoboard)", 8)
txt(14 * mm, ly - 6 * mm, "Linhas cheias: jumper MACHO-FÊMEA (fêmea no pino do sensor/ESP32, macho na trilha da protoboard). Um furo por fio.", 8)
c.showPage()

# ============================ PÁGINA 3: TABELA + CHECKLIST ============================
c.setPageSize(A4); W, H = A4
txt(W / 2, H - 16 * mm, "LIGAÇÃO FIO A FIO · CHECKLIST ANTES DE LIGAR O USB", 13, "SB", anchor="c")
rows = [("Nº", "DE", "JUMPER", "PARA", "")]
rows += [("1", "ESP32 pino 3V3", "macho-fêmea", "trilha + ESQUERDA, linha 1", "3V3"),
         ("2", "ESP32 pino G (GND)", "macho-fêmea", "trilha − ESQUERDA, linha 2", "GND"),
         ("3", "ESP32 GPIO 21 (pino S)", "macho-fêmea", "trilha + DIREITA (SDA), linha 3", "SDA"),
         ("4", "ESP32 GPIO 22 (pino S)", "macho-fêmea", "trilha − DIREITA (SCL), linha 4", "SCL")]
n = 5
linha = 6
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
y = H - 26 * mm
CC = {"3V3": RED, "GND": BLU, "SDA": GRN, "SCL": YEL}
for i, r in enumerate(rows):
    f = "SB" if i == 0 else "S"
    if i > 0 and i % 2 == 0:
        c.setFillColor(colors.HexColor("#f3f4f6")); c.rect(12 * mm, y - 1.3 * mm, 186 * mm, 4.2 * mm, fill=1, stroke=0)
    for j, v in enumerate(r[:4]):
        txt(colx[j] * mm, y, v, 7.6, f, colors.HexColor("#999999") if r[3] == "NÃO LIGAR" else colors.black)
    if r[4]:
        col = CC.get(r[4], colors.HexColor(COR.get(r[4], "#000000")))
        c.setFillColor(col); c.rect(colx[4] * mm, y - 0.6 * mm, 8 * mm, 2.6 * mm, fill=1, stroke=0)
        txt(colx[4] * mm + 10 * mm, y, r[4], 7.2, "S")
    y -= 4.25 * mm
txt(14 * mm, y - 1 * mm, "Total: 24 jumpers macho-fêmea e 5 fêmea-fêmea. Sensor usa 5 dos 6 pinos (GPIO1 fica livre).", 8, "SB")
y -= 9 * mm

check = [
    ("SB", "CHECKLIST ANTI-QUEIMA (faça na ordem)"),
    ("S", "[ ] 1. USB desconectado enquanto monta. Nenhuma fonte na entrada DC da placa de expansão."),
    ("S", "[ ] 2. Energia dos sensores só do pino 3V3 do ESP32. Nunca da fileira V, nem de 5V/VIN (a fileira V pode estar em 5 V)."),
    ("S", "[ ] 3. Trilhas da protoboard: confira com o multímetro (bipe) que a trilha vai da linha 1 à 25 sem corte."),
    ("S", "[ ] 4. Multímetro no bipe, SEM USB: + esquerda com − esquerda NÃO pode bipar (curto na alimentação)."),
    ("S", "[ ] 5. SDA com SCL NÃO pode bipar. SDA com GND e SCL com GND NÃO podem bipar."),
    ("S", "[ ] 6. Cada XSHUT vai só em GPIO (13, 14, 27, 26, 25). Nunca em 3V3, 5V ou GND."),
    ("S", "[ ] 7. Ligue o USB e ponha o dedo nos sensores e no ESP32 por 10 s: se algo esquentar, desligue na hora."),
    ("S", "[ ] 8. Multímetro em tensão contínua (V DC, escala 20 V): trilha + esquerda = 3,3 V. Trilhas SDA e SCL entre 2,5 e 3,3 V."),
    ("SB", "ORDEM DE TESTE"),
    ("S", "1. Só o FC ligado (4 fios + XSHUT). Grave o sumo_esp32 e digite  lista  e  scan : deve aparecer 0x32 FC."),
    ("S", "2. Tire o USB, acrescente o próximo sensor, ligue de novo. Repita até os 5. O scan final mostra 0x30 a 0x34."),
    ("S", "3. Aparece 0x29? Algum XSHUT está solto ou no GPIO errado."),
    ("S", "4. Digite  id  e cubra cada sensor com a mão: o nome impresso tem que bater com a etiqueta."),
    ("S", "5. Digite  ver  e mova uma caixa em volta: o ângulo e o estado mudam. Pronto para a máquina de estados."),
]
for f, l in check:
    if f == "SB": y -= 2 * mm
    txt(14 * mm, y, l, 8.2 if f == "S" else 9.5, f); y -= 4.8 * mm
c.showPage()
c.save()
print("ok", len(TOF))
