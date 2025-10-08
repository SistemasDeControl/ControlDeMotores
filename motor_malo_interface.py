# motor_malo_interface_final.py
# Interfaz final mejorada para "Motor Malo" (ISAI)
# - Tabla optimizada (QTableWidget)
# - Mini-gráficas (subplots) claras y separadas
# - Motor vectorial con humo animado (partículas) y "mal funcionamiento" visual
# - Export automático CSV + PNG al apagar (con timestamp)
# - Paleta y tipografías solicitadas
#
# Requisitos: PyQt5 matplotlib numpy pandas
# Ejecutar: python motor_malo_interface_final.py

import sys
import os
from datetime import datetime
import random
import math

import numpy as np
import pandas as pd

from PyQt5 import QtWidgets, QtCore, QtGui

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.patches as patches

# --------------------------
# Visual configuration
# --------------------------
PALETTE = {
    "bg": "#0d1117",
    "panel": "#071018",
    "accent": "#0078D7",
    "accent2": "#00C49A",
    "danger": "#FF6B6B",
    "muted": "#97A6B2",
    "text": "#e6eef3",
    "card": "#0f2a44"
}
FONT_MAIN = "DejaVu Sans"  # fallback to system font if not found

# --------------------------
# Main application
# --------------------------
class MotorMaloApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulación Motor Malo — ISAI")
        self.setMinimumSize(1200, 720)
        self.setStyleSheet(f"background-color: {PALETTE['bg']}; color: {PALETTE['text']};")

        # Simulation state
        self.motor_on = False
        self.t = 0.0
        self.max_points = 300

        # Histories
        self.time_hist = []
        self.rpm_hist = []
        self.sobre_hist = []
        self.subida_hist = []
        self.asent_hist = []
        self.error_hist = []

        # Visual state for motor
        self.wheel_angle = 0.0
        self.humo_particles = []  # list of dicts with x,y,rad,alpha,vel

        # Build UI
        self._build_ui()

        # Timer: efficient and smooth
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)  # 100ms -> 10 FPS
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    # --------------------------
    # Build UI
    # --------------------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)
        left_layout = QtWidgets.QVBoxLayout()
        right_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(left_layout, 40)
        main_layout.addLayout(right_layout, 60)

        # Left: motor canvas + controls + table
        self.fig_motor = Figure(figsize=(5,4), facecolor=PALETTE["bg"])
        self.canvas_motor = FigureCanvas(self.fig_motor)
        self.ax_motor = self.fig_motor.add_axes([0,0,1,1])
        self.ax_motor.set_facecolor(PALETTE["bg"])
        self.ax_motor.axis("off")

        left_layout.addWidget(self.canvas_motor, 68)

        # Buttons
        btn_row = QtWidgets.QHBoxLayout()
        self.btn_on = QtWidgets.QPushButton("🚀 Encender")
        self.btn_off = QtWidgets.QPushButton("🛑 Apagar")
        for b in (self.btn_on, self.btn_off):
            b.setFixedHeight(44)
            b.setFont(QtGui.QFont(FONT_MAIN, 11, QtGui.QFont.DemiBold))
            b.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.btn_on.setStyleSheet(f"background:{PALETTE['accent2']}; color: black; border-radius:8px;")
        self.btn_off.setStyleSheet(f"background:{PALETTE['danger']}; color: white; border-radius:8px;")
        btn_row.addWidget(self.btn_on)
        btn_row.addWidget(self.btn_off)
        left_layout.addLayout(btn_row)

        self.btn_on.clicked.connect(self._on_encender)
        self.btn_off.clicked.connect(self._on_apagar)

        # Table (QTableWidget) - optimized
        self.table = QtWidgets.QTableWidget(5, 2)
        self.table.setHorizontalHeaderLabels(["Métrica", "Valor"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setFocusPolicy(QtCore.Qt.NoFocus)
        self.table.setFixedHeight(240)

        metrics = ["Tiempo subida (s)", "Sobreimpulso (%)", "Tiempo asentamiento (s)", "Error estacionario", "RPM actuales"]
        for i, m in enumerate(metrics):
            item = QtWidgets.QTableWidgetItem(m)
            item.setForeground(QtGui.QColor(PALETTE["muted"]))
            item.setFont(QtGui.QFont(FONT_MAIN, 10))
            self.table.setItem(i, 0, item)
            val_item = QtWidgets.QTableWidgetItem("0.00")
            val_item.setTextAlignment(QtCore.Qt.AlignCenter)
            val_item.setFont(QtGui.QFont(FONT_MAIN, 11, QtGui.QFont.Bold))
            val_item.setForeground(QtGui.QColor(PALETTE["text"]))
            self.table.setItem(i, 1, val_item)

        # Header and column resize behavior
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.table.setStyleSheet(f"""
            QTableWidget {{ background: {PALETTE['panel']}; color: {PALETTE['text']}; font-family: '{FONT_MAIN}'; }}
            QHeaderView::section {{ background: {PALETTE['card']}; color: white; font-weight: bold; }}
        """)
        left_layout.addWidget(self.table, 32)

        # Right: big graph + mini graphs
        self.fig_main = Figure(figsize=(7,5), facecolor=PALETTE["bg"])
        self.canvas_main = FigureCanvas(self.fig_main)
        self.ax_main = self.fig_main.add_subplot(111)
        self.ax_main.set_facecolor(PALETTE["panel"])
        self.ax_main.tick_params(colors=PALETTE["text"])
        self.ax_main.set_xlabel("Tiempo (s)", color=PALETTE["text"])
        self.ax_main.set_ylabel("Valor (normalizado / unidades)", color=PALETTE["text"])
        self.ax_main.grid(True, linestyle="--", alpha=0.35)

        # Main lines
        self.line_rpm, = self.ax_main.plot([], [], label="RPM (0-100)", color=PALETTE["danger"], linewidth=2)
        self.line_sobre, = self.ax_main.plot([], [], label="Sobreimpulso (%)", color=PALETTE["accent2"], linewidth=2)
        self.line_subida, = self.ax_main.plot([], [], label="Tiempo subida (s)", color="#ffd166", linewidth=2)
        self.line_asent, = self.ax_main.plot([], [], label="Tiempo asentamiento (s)", color="#9d4edd", linewidth=2)
        self.line_error, = self.ax_main.plot([], [], label="Error estacionario", color="#ffd1dc", linewidth=2)
        self.ax_main.set_xlim(0, 10)
        self.ax_main.set_ylim(0, 110)
        self.ax_main.legend(facecolor="#06202a", edgecolor="#083", fontsize=9)

        right_layout.addWidget(self.canvas_main, 70)

        # Mini plots grid
        mini_widget = QtWidgets.QWidget()
        mini_layout = QtWidgets.QGridLayout(mini_widget)
        mini_layout.setSpacing(10)

        # Names and initial empty plot objects
        mini_names = ["RPM", "Sobreimpulso", "Tiempo subida", "Tiempo asentamiento", "Error"]
        self.mini_canvases = {}
        self.mini_lines = {}

        for idx, name in enumerate(mini_names):
            fig = Figure(figsize=(2,1.2), facecolor=PALETTE["bg"])
            canvas = FigureCanvas(fig)
            ax = fig.add_subplot(111)
            ax.set_facecolor(PALETTE["panel"])
            ax.tick_params(colors=PALETTE["text"], labelsize=8)
            ax.set_title(name, color=PALETTE["text"], fontsize=9)
            ax.grid(True, linestyle="--", alpha=0.25)
            line, = ax.plot([], [], color=PALETTE["accent2"], linewidth=1.6)
            ax.set_xlim(0, 10)
            ax.set_ylim(0, 110)
            self.mini_canvases[name] = (canvas, ax)
            self.mini_lines[name] = line
            r = idx // 3
            c = idx % 3
            mini_layout.addWidget(canvas, r, c)

        right_layout.addWidget(mini_widget, 30)

        # Initial static draw of motor
        self._draw_motor_static()

    # --------------------------
    # Draw motor (vectorial), prepare humo particles
    # --------------------------
    def _draw_motor_static(self):
        ax = self.ax_motor
        ax.clear()
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis("off")

        # Car body: rounded rectangle
        body = patches.FancyBboxPatch((0.18, 0.32), 0.64, 0.28, boxstyle="round,pad=0.02",
                                      linewidth=2, edgecolor="#2b6ea3", facecolor="#2d7fb7")
        ax.add_patch(body)
        self.car_body_patch = body

        # Hood overlay for depth
        hood = patches.Polygon([[0.18,0.32],[0.18,0.45],[0.35,0.5],[0.45,0.5],[0.45,0.6],[0.82,0.6],[0.82,0.32]],
                               closed=True, facecolor="#357ab7", edgecolor=None, alpha=0.55)
        ax.add_patch(hood)

        # Wheels
        self.wheel_left = patches.Circle((0.33, 0.28), 0.06, facecolor="#111", edgecolor="#444", lw=1.5, zorder=5)
        self.wheel_right = patches.Circle((0.67, 0.28), 0.06, facecolor="#111", edgecolor="#444", lw=1.5, zorder=5)
        ax.add_patch(self.wheel_left)
        ax.add_patch(self.wheel_right)

        # Spokes container (we draw spokes dynamically)
        self.spoke_lines = []

        # Exhaust pipe
        exhaust = patches.Rectangle((0.12, 0.46), 0.06, 0.03, facecolor="#444", edgecolor="#222")
        ax.add_patch(exhaust)
        self.exhaust = exhaust

        # Status text
        self.status_text_artist = ax.text(0.5, 0.88, "Estado: APAGADO", ha="center",
                                          color=PALETTE["danger"], fontsize=13, weight="bold")

        # Prepare humo particles (empty initially)
        self.humo_particles = []
        # We'll create a pool of particles (but invisible until motor on)
        for i in range(18):
            p = {
                "x": 0.12 + random.uniform(0.0, 0.06),
                "y": 0.48 + random.uniform(0.0, 0.06),
                "r": random.uniform(0.01, 0.03),
                "alpha": 0.0,
                "vx": random.uniform(-0.001, 0.002),
                "vy": random.uniform(0.005, 0.02)
            }
            self.humo_particles.append(p)
        # add particle patches
        self.humo_patches = []
        for p in self.humo_particles:
            c = patches.Circle((p["x"], p["y"]), p["r"], color="#bfbfbf", alpha=0.0, zorder=6)
            ax.add_patch(c)
            self.humo_patches.append(c)

        # Decorative lights
        ax.add_patch(patches.Rectangle((0.8,0.5),0.03,0.02, facecolor="#ffd166", zorder=4))
        ax.add_patch(patches.Rectangle((0.8,0.53),0.03,0.02, facecolor="#ffd166", zorder=4))

        self.canvas_motor.draw_idle()

    # --------------------------
    # Draw/rotate wheel spokes (gives impression of spin)
    # --------------------------
    def _draw_wheel_spokes(self):
        ax = self.ax_motor
        # remove old spoke lines
        for ln in self.spoke_lines:
            try:
                ln.remove()
            except Exception:
                pass
        self.spoke_lines = []
        # draw new spokes
        for center in [(0.33, 0.28), (0.67, 0.28)]:
            cx, cy = center
            for s in range(6):
                angle = math.radians(self.wheel_angle + s * (360 / 6))
                x_end = cx + 0.045 * math.cos(angle)
                y_end = cy + 0.045 * math.sin(angle)
                ln, = self.ax_motor.plot([cx, x_end], [cy, y_end], color="#aaaaaa", linewidth=1.0, zorder=7)
                self.spoke_lines.append(ln)

    # --------------------------
    # Update humo particles
    # --------------------------
    def _update_humo(self, intensity=1.0):
        # intensity 0..1 controls alpha and particle birth rate
        for i, p in enumerate(self.humo_particles):
            # when motor is on, particles are born and rise
            if self.motor_on:
                # accelerate alpha quickly then fade as moves up
                p["alpha"] = min(0.65, p["alpha"] + random.uniform(0.04, 0.12) * intensity)
                p["y"] += p["vy"] + random.uniform(0.0, 0.01) * intensity
                p["x"] += p["vx"] + random.uniform(-0.002,0.002) * intensity
                p["r"] *= (1.0 + random.uniform(0.0, 0.02) * intensity)
                # if too high, reset near exhaust
                if p["y"] > 0.86 or p["alpha"] < 0.01:
                    p["x"] = 0.12 + random.uniform(0.0, 0.06)
                    p["y"] = 0.48 + random.uniform(0.0, 0.04)
                    p["r"] = random.uniform(0.01, 0.03)
                    p["alpha"] = 0.0
            else:
                # fade out when off
                p["alpha"] = max(0.0, p["alpha"] - 0.05)
                p["y"] += 0.005
                if p["y"] > 0.86:
                    p["y"] = 0.48 + random.uniform(0.0, 0.04)
                    p["alpha"] = 0.0
            # update patch
            try:
                self.humo_patches[i].center = (p["x"], p["y"])
                self.humo_patches[i].radius = p["r"]
                self.humo_patches[i].set_alpha(p["alpha"])
            except Exception:
                pass

    # --------------------------
    # Tick handler: update simulation values and visuals
    # --------------------------
    def _tick(self):
        if self.motor_on:
            self.t += 0.1

            # generate data (bounded)
            rpm = float(np.clip(60 + random.uniform(-30, 30), 0, 100))
            sobre = float(np.clip(random.gauss(15, 6), 0, 100))
            subida = float(np.clip(random.uniform(0.6, 3.5), 0, 10))
            asent = float(np.clip(random.uniform(1.8, 7.5), 0, 20))
            error = float(np.clip(random.uniform(0.0, 0.25), 0, 1.0))

            # append histories
            self.time_hist.append(self.t)
            self.rpm_hist.append(rpm)
            self.sobre_hist.append(sobre)
            self.subida_hist.append(subida)
            self.asent_hist.append(asent)
            self.error_hist.append(error)

            # simple smoothing: moving average for display (optional)
            if len(self.rpm_hist) > 1:
                window = 3
                smoothed = np.convolve(self.rpm_hist, np.ones(window)/window, mode='same')
            else:
                smoothed = np.array(self.rpm_hist)

            # crop history lengths
            if len(self.time_hist) > self.max_points:
                for arr in (self.time_hist, self.rpm_hist, self.sobre_hist, self.subida_hist, self.asent_hist, self.error_hist):
                    arr.pop(0)

            # update table
            vals = [subida, sobre, asent, error, rpm]
            for i, v in enumerate(vals):
                it = QtWidgets.QTableWidgetItem(f"{v:.2f}")
                it.setTextAlignment(QtCore.Qt.AlignCenter)
                it.setForeground(QtGui.QColor(PALETTE["text"]))
                it.setFont(QtGui.QFont(FONT_MAIN, 11, QtGui.QFont.Bold))
                self.table.setItem(i, 1, it)

            # update main plot lines (normalize some metrics for shared axis)
            self.line_rpm.set_data(self.time_hist, self.rpm_hist)
            self.line_sobre.set_data(self.time_hist, self.sobre_hist)
            subida_norm = [min(100, (x / 5.0) * 100.0) for x in self.subida_hist]
            asent_norm = [min(100, (x / 10.0) * 100.0) for x in self.asent_hist]
            error_norm = [min(100, x * 100.0) for x in self.error_hist]
            self.line_subida.set_data(self.time_hist, subida_norm)
            self.line_asent.set_data(self.time_hist, asent_norm)
            self.line_error.set_data(self.time_hist, error_norm)

            # update main axes
            self.ax_main.set_xlim(max(0, self.t - 10), self.t)
            self.ax_main.set_ylim(0, 110)
            self.canvas_main.draw_idle()

            # update mini graphs
            mini_map = {
                "RPM": (self.rpm_hist, PALETTE["danger"]),
                "Sobreimpulso": (self.sobre_hist, PALETTE["accent2"]),
                "Tiempo subida": (subida_norm, "#ffd166"),
                "Tiempo asentamiento": (asent_norm, "#9d4edd"),
                "Error": (error_norm, "#ffd1dc"),
            }
            for name, (canvas, ax) in self.mini_canvases.items():
                arr = mini_map[name][0]
                line = self.mini_lines[name]
                line.set_data(self.time_hist, arr)
                ax.set_xlim(max(0, self.t - 10), self.t)
                ax.set_ylim(0, 110)
                canvas.draw_idle()

            # update wheel rotation speed proportional to rpm
            self.wheel_angle += rpm * 0.12
            self._draw_wheel_spokes()

            # update humo with intensity based on error (more error => thicker smoke)
            intensity = min(1.0, 1.0 if error > 0.12 else 0.6)
            self._update_humo(intensity=intensity)

            # change car body color to indicate ON with slight pulsing if error high
            pulse = 1.0 + 0.02 * math.sin(self.t * 4.0) if error > 0.15 else 1.0
            if error > 0.15:
                # slight hue shift to warn
                for p in self.ax_motor.patches:
                    if isinstance(p, patches.FancyBboxPatch):
                        p.set_facecolor(PALETTE["accent2"])
            else:
                for p in self.ax_motor.patches:
                    if isinstance(p, patches.FancyBboxPatch):
                        p.set_facecolor("#2d7fb7")

            # status text
            self.status_text_artist.set_text(f"Estado: ENCENDIDO  (err={error:.2f})")
            self.status_text_artist.set_color(PALETTE["accent2"])

            # final draw motor canvas
            self.canvas_motor.draw_idle()

        else:
            # Motor off: fade smoke and reset small visuals
            self._update_humo(intensity=0.0)
            for p in self.ax_motor.patches:
                if isinstance(p, patches.FancyBboxPatch):
                    p.set_facecolor("#2d7fb7")
            self.status_text_artist.set_text("Estado: APAGADO")
            self.status_text_artist.set_color(PALETTE["danger"])
            # mini canvases stay as last frame; draw motor canvas
            self.canvas_motor.draw_idle()

    # --------------------------
    # Draw wheel spokes (rotating) - small optimization
    # --------------------------
    def _draw_wheel_spokes(self):
        # remove old spoke lines
        for ln in list(self.ax_motor.lines):
            try:
                if getattr(ln, "is_spoke", False):
                    ln.remove()
            except Exception:
                pass
        # draw new spokes
        for center in [(0.33, 0.28), (0.67, 0.28)]:
            cx, cy = center
            for s in range(6):
                angle = math.radians(self.wheel_angle + s * (360 / 6))
                x_end = cx + 0.045 * math.cos(angle)
                y_end = cy + 0.045 * math.sin(angle)
                ln, = self.ax_motor.plot([cx, x_end], [cy, y_end], color="#aaaaaa", linewidth=1.0, zorder=7)
                ln.is_spoke = True  # mark for removal next tick

    # --------------------------
    # Handlers: encender/apagar (with export on stop)
    # --------------------------
    def _on_encender(self):
        self.motor_on = True

    def _on_apagar(self):
        was_on = self.motor_on
        self.motor_on = False
        if was_on:
            # export CSV and PNG with timestamp
            self._export_data_and_figure()

    # --------------------------
    # Export function (CSV + PNG)
    # --------------------------
    def _export_data_and_figure(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        outdir = os.path.join(os.getcwd(), "exports")
        os.makedirs(outdir, exist_ok=True)
        csv_path = os.path.join(outdir, f"tabla_datos_{timestamp}.csv")
        png_path = os.path.join(outdir, f"grafico_rendimiento_{timestamp}.png")

        # compose dataframe
        df = pd.DataFrame({
            "t": self.time_hist,
            "rpm": self.rpm_hist,
            "sobreimpulso": self.sobre_hist,
            "tiempo_subida": self.subida_hist,
            "tiempo_asentamiento": self.asent_hist,
            "error_estacionario": self.error_hist
        })
        try:
            df.to_csv(csv_path, index=False)
            # save main figure
            self.fig_main.savefig(png_path, dpi=200, facecolor=self.fig_main.get_facecolor())
            QtWidgets.QMessageBox.information(self, "Exportado",
                                              f"Exportados:\n{csv_path}\n{png_path}")
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Error exportando", str(e))

# --------------------------
# Run application
# --------------------------
def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MotorMaloApp()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
