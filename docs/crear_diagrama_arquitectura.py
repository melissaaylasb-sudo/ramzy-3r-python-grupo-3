"""Genera una lámina 16:9 de la arquitectura y trazabilidad del proyecto 3R."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle


HERE = Path(__file__).resolve().parent
NAVY = "#02244A"
PETROL = "#004B62"
BLUE = "#4A91BB"
SKY = "#A5D0F0"
GOLD = "#C48742"
BG = "#F2F7FA"
INK = "#12364D"
MUTED = "#496A7D"
LINE = "#D5E3EC"


fig = plt.figure(figsize=(16, 9), facecolor=BG)
ax = fig.add_axes((0, 0, 1, 1))
ax.set_xlim(0, 16)
ax.set_ylim(0, 9)
ax.axis("off")


def label(x, y, value, *, size=10, color=INK, weight="normal", va="top", ha="left"):
    return ax.text(
        x, y, value, ha=ha, va=va, fontsize=size, color=color,
        fontweight=weight, fontfamily="DejaVu Sans", linespacing=1.4,
    )


def card(x, y, width, height, *, kicker, title, body, accent=BLUE, fill="white",
         title_size=12.2, body_size=9.9):
    ax.add_patch(FancyBboxPatch(
        (x, y), width, height, boxstyle="round,pad=0.012,rounding_size=0.13",
        linewidth=1.1, edgecolor=LINE, facecolor=fill,
    ))
    ax.add_patch(Rectangle((x + 0.02, y + height - 0.09), width - 0.04, 0.07,
                           linewidth=0, facecolor=accent))
    label(x + 0.2, y + height - 0.22, kicker, size=8.2, color=accent, weight="bold")
    label(x + 0.2, y + height - 0.54, title, size=title_size, color=NAVY, weight="bold")
    label(x + 0.2, y + height - 0.9, body, size=body_size, color=MUTED)


def arrow(start, end, color=BLUE, width=1.8):
    ax.add_patch(FancyArrowPatch(
        start, end, arrowstyle="-|>", mutation_scale=15,
        linewidth=width, color=color, shrinkA=0, shrinkB=0,
        connectionstyle="arc3,rad=0",
    ))


# Cabecera y leyenda: el color marca la procedencia de la evidencia.
ax.add_patch(Rectangle((0, 8.08), 16, 0.92, linewidth=0, facecolor=NAVY))
label(0.52, 8.82, "DEL ARTÍCULO AL SIMULADOR 3R EN PYTHON",
      size=22.2, color="white", weight="bold")
label(0.54, 8.32, "Arquitectura y trazabilidad científica · Grupo 3 · MIA-12",
      size=11.2, color=SKY)
ax.add_patch(Rectangle((11.8, 8.43), 0.16, 0.16, facecolor=GOLD, linewidth=0))
label(12.04, 8.55, "publicado", size=9.2, color="white", va="center")
ax.add_patch(Rectangle((13.37, 8.43), 0.16, 0.16, facecolor=BLUE, linewidth=0))
label(13.61, 8.55, "desarrollo propio", size=9.2, color="white", va="center")

for x, heading in [
    (0.53, "01  ARTÍCULO"),
    (4.35, "02  MODELO PYTHON"),
    (8.94, "03  EJECUCIÓN"),
    (12.04, "04  SALIDAS Y VISOR"),
]:
    label(x, 7.84, heading, size=10.3, color=PETROL, weight="bold")

card(0.52, 4.65, 3.32, 2.65,
     kicker="BASE CIENTÍFICA PUBLICADA", title="Modelo de Ramzy et al.",
     body="Tabla 1 · DH del manipulador 3R\n"
          "Ecs. (6–12) · directa e inversa\n"
          "Ecs. (13–18) · dinámica\n"
          "Tabla 2 · parámetros físicos",
     accent=GOLD, body_size=10.3)
card(0.52, 2.1, 3.32, 2.08,
     kicker="EVIDENCIA FÍSICA PUBLICADA", title="Prototipo VSL · 2 bar",
     body="Tablas 3 y 4: mediciones\n"
          "transcritas en data/tabla3.csv\n"
          "y data/tabla4.csv",
     accent=GOLD, body_size=10.2)

card(4.35, 5.9, 4.13, 1.4,
     kicker="RECONSTRUCCIÓN DEL MODELO", title="ramzy3r/kinematics.py",
     body="DH, FK/IK, Jacobiano y singularidades\n"
          "FK: ángulos → XYZ  ·  IK: XYZ → ángulos",
     accent=PETROL, title_size=12, body_size=9.7)
card(4.35, 4.4, 4.13, 1.25,
     kicker="SIMULACIÓN PROPIA", title="ramzy3r/control.py",
     body="3 escenarios × 2 métodos: pseudoinversa / DLS",
     accent=BLUE, title_size=12, body_size=9.7)
card(4.35, 2.9, 4.13, 1.25,
     kicker="RECONSTRUCCIÓN CON SUPUESTOS", title="ramzy3r/dynamics.py",
     body="Masa, gravedad, rigidez y respuesta libre",
     accent=BLUE, title_size=12, body_size=9.7)
card(4.35, 1.4, 4.13, 1.25,
     kicker="AUDITORÍA DOCUMENTAL", title="ramzy3r/benchmarks.py",
     body="Compara Tabla 3; recalcula muestra de Tabla 4",
     accent=PETROL, title_size=12, body_size=9.6)

card(8.95, 4.7, 2.7, 2.6,
     kicker="ORQUESTADOR", title="run_experiments.py",
     body="Ejecuta cinemática\ny singularidades,\ncontrol, dinámica y\ncomparación con tablas.",
     accent=BLUE, title_size=11.2, body_size=10.0)
card(8.95, 2.9, 2.7, 1.25,
     kicker="COMPROBACIÓN", title="12 pruebas",
     body="tests/test_model.py",
     accent=PETROL, title_size=12.2, body_size=9.9)
card(8.95, 1.4, 2.7, 1.25,
     kicker="IMPLEMENTACIÓN", title="Python independiente",
     body="El artículo usa MATLAB/Simulink.",
     accent=PETROL, title_size=11.4, body_size=9.4)

card(12.05, 5.9, 3.4, 1.4,
     kicker="EJECUCIÓN EN ESTE EQUIPO", title="resultados_locales*/",
     body="resumen.json · 16 CSV · 9 PNG",
     accent=BLUE, title_size=12.2, body_size=10.0)
card(12.05, 3.95, 3.4, 1.45,
     kicker="VISUALIZACIÓN INTERACTIVA", title="launch_viewer.py + viewer/",
     body="Three.js: 6 trayectorias de control\ntelemetría y guía de exposición",
     accent=PETROL, title_size=11.6, body_size=9.7)
card(12.05, 2.0, 3.4, 1.45,
     kicker="EVIDENCIA FUERA DEL VISOR", title="CSV y gráficas",
     body="Inversa, singularidades, dinámica\ny comparación con Tablas 3–4",
     accent=PETROL, title_size=12, body_size=9.7)

# Dos entradas del artículo: teoría/mediciones. El visor es una rama de las salidas.
arrow((3.85, 6.61), (4.27, 6.61), color=GOLD)
arrow((3.85, 2.38), (4.27, 2.02), color=GOLD)
arrow((8.49, 5.68), (8.88, 5.68))
arrow((11.66, 6.6), (11.99, 6.6))
arrow((13.76, 5.83), (13.76, 5.47), color=PETROL)

ax.add_patch(FancyBboxPatch(
    (0.52, 0.22), 14.93, 0.86, boxstyle="round,pad=0.012,rounding_size=0.12",
    linewidth=0, facecolor=NAVY,
))
label(0.74, 0.91, "ALCANCE", size=9.3, color=SKY, weight="bold")
label(2.02, 0.91,
      "El visor anima el modelo nominal, no el prototipo flexible; control y dinámica Python son ensayos propios.",
      size=10.1, color="white")
label(2.02, 0.58,
      "La Tabla 4 no contiene la serie completa para reproducir el experimento físico.",
      size=9.7, color=SKY)
label(15.44, 0.1, "Ramzy, Hammad y Maged · Applied Sciences 14(12), 5285 · DOI 10.3390/app14125285",
      size=7.9, color=MUTED, va="bottom", ha="right")

for extension in ("png", "svg", "pdf"):
    fig.savefig(HERE / f"arquitectura_ramzy_3r.{extension}", dpi=180,
                facecolor=BG, edgecolor="none")
plt.close(fig)
