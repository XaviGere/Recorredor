"""Genera un diagrama de wiring LIMPIO (routing ortogonal + riels de potencia)
del circuito Recorredor: Arduino Nano + L293D + SG90 + 2xHC-SR04 + 2 motores DC.

Salida: docs/recorredor_tinkercad_nano_circuito.png
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, Circle, FancyArrow

fig, ax = plt.subplots(figsize=(16, 10), dpi=110)
ax.set_xlim(0, 16); ax.set_ylim(0, 10); ax.axis("off")
fig.patch.set_facecolor("white")

C_VCC, C_GND, C_6V = "#e03434", "#222222", "#b34700"
C_SIG, C_US, C_MOT, C_IN = "#e8862e", "#2f7fd0", "#2e9e4f", "#9b59b6"


def wire(pts, color, lw=2.0):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=color, lw=lw, solid_capstyle="round",
            solid_joinstyle="round", zorder=3)


def box(x, y, w, h, title, sub, fc, title_fs=10):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.07",
                                fc=fc, ec="#444", lw=1.6, zorder=2))
    ax.text(x + w / 2, y + h - 0.30, title, ha="center", fontsize=title_fs,
            weight="bold", color="#222", zorder=4)
    ax.text(x + w / 2, y + 0.24, sub, ha="center", fontsize=7.5, color="#555", zorder=4)


# ============================================================ PROTOBOARD
ax.add_patch(FancyBboxPatch((2.8, 3.3), 7.6, 5.3, boxstyle="round,pad=0.06",
                            fc="#fdfdf4", ec="#c9c9b6", lw=2, zorder=1))
for gx in range(25):
    for gy in range(7):
        ax.add_patch(Circle((3.05 + gx * 0.29, 3.7 + gy * 0.72), 0.038,
                            fc="#dcdcca", ec="none", zorder=1))
ax.text(3.0, 8.45, "PROTOBOARD", fontsize=9, color="#999", weight="bold")

# ============================================================ ARDUINO NANO
ax.add_patch(FancyBboxPatch((3.3, 3.8), 1.15, 4.4, boxstyle="round,pad=0.03",
                            fc="#1b4f9c", ec="#0d2f61", lw=2, zorder=2))
ax.text(3.875, 7.85, "ARDUINO\nNANO", ha="center", va="center", fontsize=9.5,
        color="white", weight="bold", zorder=4)
ax.add_patch(mpatches.Rectangle((3.72, 7.15), 0.31, 0.28, fc="#c8c8c8",
                                ec="#555", zorder=4))  # USB

# pines: (nombre, y, lado)
nano_L = [("D10", 7.4), ("D11", 7.05), ("A2", 6.4), ("A3", 6.0),
          ("A0", 5.0), ("D2", 4.5), ("GND", 4.0)]
nano_R = [("D9", 7.5), ("D12", 7.1), ("D8", 6.65), ("D7", 6.2), ("D6", 5.75),
          ("D5", 5.3), ("D4", 4.85), ("5V", 4.2)]
NP = {}
for n, y in nano_L:
    ax.add_patch(mpatches.Rectangle((3.22, y - 0.08), 0.15, 0.16, fc="#e8c35a",
                                    ec="#997a1a", zorder=4))
    ax.text(3.16, y, n, ha="right", va="center", fontsize=7.5, weight="bold")
    NP[n] = (3.22, y)
for n, y in nano_R:
    ax.add_patch(mpatches.Rectangle((4.38, y - 0.08), 0.15, 0.16, fc="#e8c35a",
                                    ec="#997a1a", zorder=4))
    ax.text(4.59, y, n, ha="left", va="center", fontsize=7.5, weight="bold")
    NP[n] = (4.53, y)

# ============================================================ L293D
ax.add_patch(mpatches.Rectangle((6.7, 4.1), 1.15, 2.7, fc="#333", ec="#111",
                                lw=1.5, zorder=2))
ax.add_patch(mpatches.Arc((7.275, 6.8), 0.34, 0.34, theta1=270, theta2=450,
                          fc="#fdfdf4", ec="none", zorder=3))
ax.text(7.275, 5.45, "L293D", ha="center", rotation=90, fontsize=9,
        color="white", weight="bold", zorder=4)
# izq: 1..8 arriba->abajo ; der: 16..9
lp = [("1 EN", 6.5), ("2 IN1", 6.2), ("3 OUT1", 5.9), ("4 GND", 5.6),
      ("5 GND", 5.3), ("6 OUT2", 5.0), ("7 IN2", 4.7), ("8 VS", 4.4)]
rp = [("16 VSS", 6.5), ("15 IN3", 6.2), ("14 OUT3", 5.9), ("13 GND", 5.6),
      ("12 GND", 5.3), ("11 OUT4", 5.0), ("10 IN4", 4.7), ("9 EN", 4.4)]
LP, RP = {}, {}
for n, y in lp:
    ax.add_patch(mpatches.Rectangle((6.58, y - 0.07), 0.14, 0.14, fc="#bbb",
                                    ec="#666", zorder=4))
    ax.text(6.52, y, n, ha="right", va="center", fontsize=6.6)
    LP[int(n.split()[0])] = (6.58, y)
for n, y in rp:
    ax.add_patch(mpatches.Rectangle((7.83, y - 0.07), 0.14, 0.14, fc="#bbb",
                                    ec="#666", zorder=4))
    ax.text(8.0, y, n, ha="left", va="center", fontsize=6.6)
    RP[int(n.split()[0])] = (7.97, y)

# ============================================================ COMPONENTES
box(0.25, 7.5, 2.3, 1.0, "HC-SR04 IZQ", "VCC  TRIG  ECHO  GND", "#9fd8f0")
for i, cx in enumerate((0.55, 1.05, 1.55, 2.05)):
    ax.add_patch(Circle((cx, 7.85), 0.12, fc="white", ec="#4a90b8", zorder=4))
box(0.25, 5.5, 2.3, 1.0, "HC-SR04 DER", "VCC  TRIG  ECHO  GND", "#9fd8f0")
for cx in (0.55, 1.05, 1.55, 2.05):
    ax.add_patch(Circle((cx, 5.85), 0.12, fc="white", ec="#4a90b8", zorder=4))
box(0.25, 3.5, 2.3, 1.0, "BUMPER", "contacto → GND (opc.)", "#e6b8d8")
box(0.25, 1.6, 2.3, 1.0, "BOTÓN START", "a GND (opc.)", "#f2e2a8")

box(12.6, 7.3, 3.2, 1.3, "SERVO SG90", "señal  |  VCC  |  GND", "#f9c58a")
ax.add_patch(Circle((13.3, 8.35), 0.30, fc="#e08a2e", ec="#8a5118", zorder=4))
ax.text(13.3, 8.35, "90°", ha="center", va="center", fontsize=6.5,
        color="white", weight="bold", zorder=4)
box(12.6, 5.1, 3.2, 1.2, "MOTOR DC IZQ", "ruedas izquierda(s)", "#c9c9c9")
box(12.6, 3.4, 3.2, 1.2, "MOTOR DC DER", "ruedas derecha(s)", "#c9c9c9")
box(12.6, 1.3, 3.2, 1.3, "BATERÍA 4×AA", "+ 6V  |  − GND", "#b8e6b0")

# ============================================================ RIELS
ax.plot([0.4, 15.8], [0.95, 0.95], color=C_VCC, lw=3, zorder=2)
ax.text(0.42, 1.12, "RIEL 5 V", fontsize=8, color=C_VCC, weight="bold")
ax.plot([0.4, 15.8], [0.45, 0.45], color=C_GND, lw=3, zorder=2)
ax.text(0.42, 0.60, "RIEL GND", fontsize=8, color=C_GND, weight="bold")

def tap(x0, y0, rail_y, color):
    wire([(x0, y0), (x0, rail_y)], color, lw=1.8)
    ax.add_patch(Circle((x0, rail_y), 0.06, fc=color, ec="none", zorder=4))

# ============================================================ SEÑALES IZQ
wire([NP["D10"], (2.85, NP["D10"][1]), (2.85, 8.1), (1.05, 8.1), (1.05, 8.5)], C_US)
wire([NP["D11"], (2.70, NP["D11"][1]), (2.70, 7.95), (1.55, 7.95), (1.55, 8.5)], C_US)
wire([NP["A2"], (2.85, NP["A2"][1]), (2.85, 6.9), (1.05, 6.9), (1.05, 6.5)], C_US)
wire([NP["A3"], (2.70, NP["A3"][1]), (2.70, 6.75), (1.55, 6.75), (1.55, 6.5)], C_US)
wire([NP["D2"], (2.55, NP["D2"][1]), (2.55, 4.05), (2.55, 4.05)], C_IN)
wire([NP["D2"], (2.55, 4.05)], C_IN)  # ya queda al lado de la caja bumper
wire([NP["A0"], (2.45, NP["A0"][1]), (2.45, 2.1), (2.55, 2.1)], C_IN)

# ============================================================ SEÑALES DER
wire([NP["D5"], (5.0, 5.3), (5.0, 6.5), LP[1]], C_SIG)
wire([NP["D4"], (5.25, 4.85), (5.25, 6.2), LP[2]], C_SIG)
wire([NP["D7"], (5.5, 6.2), (5.5, 4.7), LP[7]], C_SIG)
# D12/D8/D6 -> lado derecho del chip (por el canal superior e inferior)
wire([NP["D12"], (6.0, 7.1), (6.0, 7.2), (8.15, 7.2), (8.15, 6.2), RP[15]], C_SIG)
wire([NP["D8"], (6.25, 6.65), (6.25, 7.05), (8.4, 7.05), (8.4, 4.7), RP[10]], C_SIG)
wire([NP["D6"], (5.75, 5.75), (5.75, 3.95), (8.6, 3.95), (8.6, 4.4), RP[9]], C_SIG)
# D9 -> servo (por encima de todo, línea casi recta)
wire([NP["D9"], (12.6, 8.15)], C_SIG)

# ============================================================ MOTORES
wire([LP[3], (6.35, 5.9), (6.35, 7.35), (12.35, 7.35), (12.35, 6.05), (12.6, 6.05)], C_MOT)
wire([LP[6], (6.45, 5.0), (6.45, 7.15), (11.6, 7.15), (11.6, 5.55), (12.6, 5.55)], C_MOT)
wire([RP[14], (8.2, 5.9), (8.2, 5.6), (11.9, 5.6), (11.9, 4.4), (12.6, 4.4)], C_MOT)
wire([RP[11], (8.5, 5.0), (8.5, 4.95), (11.5, 4.95), (11.5, 3.95), (12.6, 3.95)], C_MOT)

# ============================================================ POTENCIA
tap(NP["5V"][0] + 0.15, 4.2, 0.95, C_VCC)                        # Nano 5V -> riel
wire([LP[8], (6.3, 4.4), (6.3, 2.1), (12.35, 2.1), (12.35, 2.4), (12.6, 2.4)], C_6V, lw=2.2)  # VS -> bat+
wire([LP[4], (6.15, 5.6), (6.15, 0.45)], C_GND)                  # GNDs chip
wire([LP[5], (5.95, 5.3), (5.95, 0.45)], C_GND)
wire([RP[16], (8.05, 6.5), (8.05, 0.95)], C_VCC)                 # VSS -> riel 5V
wire([RP[13], (9.0, 5.6), (9.0, 0.45)], C_GND)
wire([NP["GND"], (2.95, 4.0), (2.95, 0.45)], C_GND)              # GND Nano
# bateria
wire([(15.2, 2.6), (15.2, 0.45)], C_GND)                         # bat- -> riel GND
# sensores y servo a riel
tap(0.55, 7.5, 0.95, C_VCC);  tap(2.05, 7.5, 0.45, C_GND)        # US1
tap(0.55, 5.5, 0.95, C_VCC);  tap(2.05, 5.5, 0.45, C_GND)        # US2
tap(14.2, 7.3, 0.95, C_VCC);  tap(15.0, 7.3, 0.45, C_GND)        # servo
tap(1.6, 1.6, 0.45, C_GND)                                       # botón -> GND
tap(1.6, 3.5, 0.45, C_GND)                                       # bumper -> GND

# ============================================================ ETIQUETAS
ax.text(11.2, 7.45, "OUT1/OUT2 → motor IZQ", fontsize=7, color=C_MOT, ha="right")
ax.text(10.5, 4.65, "OUT3/OUT4 → motor DER", fontsize=7, color=C_MOT, ha="right")
ax.text(9.6, 2.0, "VS ← +6V batería", fontsize=7, color=C_6V, ha="center")

# ============================================================ LEYENDA
lx, ly = 0.35, 9.0
ax.text(lx, ly + 0.75, "LEYENDA", fontsize=9, weight="bold")
for i, (c, t) in enumerate([(C_VCC, "5 V (lógica)"), (C_6V, "6 V motores"),
                            (C_GND, "GND común"), (C_SIG, "Señal / control"),
                            (C_US, "HC-SR04 TRIG/ECHO"), (C_MOT, "Salida motores"),
                            (C_IN, "Entradas")]):
    ax.plot([lx, lx + 0.4], [ly + 0.4 - i * 0.22] * 2, color=c, lw=3)
    ax.text(lx + 0.5, ly + 0.4 - i * 0.22, t, fontsize=7.5, va="center")

ax.text(8.0, 9.65, "RECORREDOR — Circuito Tinkercad: Nano + L293D + SG90 + 2×HC-SR04 + 2 motores DC",
        ha="center", fontsize=13, weight="bold", color="#1b4f9c")
ax.text(8.0, 0.02, "3 ruedas = 2 motores + rueda loca  |  4 ruedas = 2 motores por canal  |  Servo: dirección o torreta  |  VSS=5V lógica, VS=6V motores, GND común",
        ha="center", fontsize=8, color="#666")

plt.savefig("docs/recorredor_tinkercad_nano_circuito.png",
            bbox_inches="tight", facecolor="white")
print("OK")
