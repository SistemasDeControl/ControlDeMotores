# motor_malo_pid.py
"""
Motor Malo - Interfaz y Modelo (ISAI) - Versión mejorada
- Correcciones de bugs, estilos visuales y legibilidad
- RPM cap en gráficas a 0-100
- Tabla y controles con mejor tipografía y tamaños
- Mini-gráficas de métricas
- Exportación de series temporales + métricas (CSV) y figura (PNG)

Requisitos:
    pip install PyQt5 matplotlib numpy pandas scipy control
(control es opcional; si no está instalado se usa scipy.signal)

Ejecución:
    python motor_malo_pid.py
"""

import os
import sys
import math
import random
from datetime import datetime

import numpy as np
import pandas as pd

from PyQt5 import QtWidgets, QtCore, QtGui

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.patches as patches

# Prefer control, fallback to scipy.signal
try:
    import control as ctl
    USE_CONTROL = True
except Exception:
    from scipy import signal
    USE_CONTROL = False

# -----------------------
# Defaults / parameters
# -----------------------
DEFAULT_K = 0.6
DEFAULT_TAU = 1.2
# default target shown in the UI (user can change) - graphs always clamp to 0..100
DEFAULT_TARGET_RPM = 50
DEFAULT_SIM_TIME = 8.0

# Visual palette (requested)
PALETTE = {
    "bg": "#0d1117",
    "panel": "#071018",
    "accent": "#0078D7",
    "accent2": "#00C49A",
    "danger": "#FF6B6B",
    "text": "#e6eef3",
    "card": "#0f2a44"
}
FONT = "DejaVu Sans"

# -----------------------
# Control/model helpers
# -----------------------

def make_first_order_system(K, tau):
    """Return TransferFunction G(s) = K/(tau*s + 1)."""
    if USE_CONTROL:
        return ctl.TransferFunction([K], [tau, 1])
    else:
        return signal.TransferFunction([K], [tau, 1])


def simulate_step_response(sys, T):
    """Simulate unit step response on time vector T."""
    if USE_CONTROL:
        t, y = ctl.step_response(sys, T=T)
    else:
        t, y = signal.step(sys, T=T)
    return np.asarray(t), np.asarray(y)


def simulate_forced_response(sys, T, u):
    """Simulate response to input u(t) on time vector T."""
    if USE_CONTROL:
        t, y, _ = ctl.forced_response(sys, T=T, U=u)
    else:
        t, y, _ = signal.lsim(sys, U=u, T=T)
    return np.asarray(t), np.asarray(y)


def compute_metrics(t, y, ref):
    """Compute rise time (10-90%), overshoot (%), settling time (2%), steady-state error.
    Returns dict with float values (NaN if not computable).
    """
    n = len(y)
    if n == 0:
        return {"t_rise": np.nan, "overshoot": np.nan, "t_settle": np.nan, "ess": np.nan}

    # steady-state: mean of last 5% samples to reduce noise
    last_n = max(1, int(0.05 * n))
    y_ss = np.mean(y[-last_n:])

    # overshoot
    if ref != 0:
        peak = np.max(y)
        overshoot = max(0.0, (peak - ref) / abs(ref) * 100.0)
    else:
        overshoot = np.nan

    # rise time (10% - 90% of ref)
    try:
        low = 0.1 * ref
        high = 0.9 * ref
        idx_low = np.where(y >= low)[0]
        idx_high = np.where(y >= high)[0]
        if len(idx_low) and len(idx_high):
            t_low = t[idx_low[0]]
            t_high = t[idx_high[0]]
            t_rise = t_high - t_low if t_high >= t_low else np.nan
        else:
            t_rise = np.nan
    except Exception:
        t_rise = np.nan

    # settling time: first time after which response remains within 2% band for remainder
    try:
        band = 0.02 * abs(ref)
        t_settle = np.nan
        for i in range(len(t)):
            if np.all(np.abs(y[i:] - ref) <= band):
                t_settle = t[i]
                break
    except Exception:
        t_settle = np.nan

    ess = abs(ref - y_ss)
    return {"t_rise": float(t_rise) if not np.isnan(t_rise) else np.nan,
            "overshoot": float(overshoot) if not np.isnan(overshoot) else np.nan,
            "t_settle": float(t_settle) if not np.isnan(t_settle) else np.nan,
            "ess": float(ess) if not np.isnan(ess) else np.nan}


# -----------------------
# Main GUI application
# -----------------------
class MotorMaloModelGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Malo - Modelo y Simulación (ISAI)")
        self.setMinimumSize(1260, 820)
        self.setStyleSheet(f"background-color: {PALETTE['bg']}; color: {PALETTE['text']};")

        # state
        self.K = DEFAULT_K
        self.tau = DEFAULT_TAU
        self.target_rpm = DEFAULT_TARGET_RPM
        self.sim_time = DEFAULT_SIM_TIME
        self.dt = 0.01
        self.motor_anim_on = False

        # histories
        self.t_step = np.array([])
        self.y_step = np.array([])
        self.t_rpm = np.array([])
        self.y_rpm = np.array([])

        # latest metrics
        self.latest_metrics = None

        # build UI
        self._build_ui()

        # animation timer for motor/humo
        self.anim_timer = QtCore.QTimer(self)
        self.anim_timer.setInterval(80)  # 12.5 FPS
        self.anim_timer.timeout.connect(self._animate_motor)
        self.anim_timer.start()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        # left column: motor visual + controls + export
        left_col = QtWidgets.QVBoxLayout()
        main_layout.addLayout(left_col, 36)

        # right column: plots + metrics
        right_col = QtWidgets.QVBoxLayout()
        main_layout.addLayout(right_col, 64)

        # ---------- motor visual (matplotlib canvas) ----------
        self.fig_motor = Figure(figsize=(5,3), facecolor=PALETTE["bg"])
        self.canvas_motor = FigureCanvas(self.fig_motor)
        self.ax_motor = self.fig_motor.add_axes([0, 0, 1, 1])
        self.ax_motor.set_xlim(0, 1)
        self.ax_motor.set_ylim(0, 1)
        self.ax_motor.axis("off")
        self.ax_motor.set_facecolor(PALETTE["bg"])
        left_col.addWidget(self.canvas_motor, 62)
        self._setup_motor_visual_elements()

        # ---------- controls group ----------
        grp = QtWidgets.QGroupBox("Parámetros y Control")
        grp.setStyleSheet(f"QGroupBox {{ color: {PALETTE['text']}; font-weight:bold; }}")
        left_col.addWidget(grp)
        g_layout = QtWidgets.QGridLayout(grp)

        label_font = QtGui.QFont(FONT, 11)
        value_font = QtGui.QFont(FONT, 11, QtGui.QFont.Bold)

        # K
        lbl_k = QtWidgets.QLabel("K (ganancia)")
        lbl_k.setFont(label_font)
        g_layout.addWidget(lbl_k, 0, 0)
        self.spin_K = QtWidgets.QDoubleSpinBox()
        self.spin_K.setRange(0.01, 100.0)
        self.spin_K.setSingleStep(0.05)
        self.spin_K.setValue(self.K)
        self.spin_K.setFont(value_font)
        self.spin_K.valueChanged.connect(self._param_changed)
        g_layout.addWidget(self.spin_K, 0, 1)

        # tau
        lbl_tau = QtWidgets.QLabel("τ (s)")
        lbl_tau.setFont(label_font)
        g_layout.addWidget(lbl_tau, 1, 0)
        self.spin_tau = QtWidgets.QDoubleSpinBox()
        self.spin_tau.setRange(0.01, 10.0)
        self.spin_tau.setSingleStep(0.05)
        self.spin_tau.setValue(self.tau)
        self.spin_tau.setFont(value_font)
        self.spin_tau.valueChanged.connect(self._param_changed)
        g_layout.addWidget(self.spin_tau, 1, 1)

        # target rpm (LIMITED to 0..100 for display legibility)
        lbl_ref = QtWidgets.QLabel("Referencia RPM (0-100)")
        lbl_ref.setFont(label_font)
        g_layout.addWidget(lbl_ref, 2, 0)
        self.spin_ref = QtWidgets.QSpinBox()
        self.spin_ref.setRange(0, 100)
        self.spin_ref.setSingleStep(1)
        self.spin_ref.setValue(self.target_rpm)
        self.spin_ref.setFont(value_font)
        self.spin_ref.valueChanged.connect(self._param_changed)
        g_layout.addWidget(self.spin_ref, 2, 1)

        # sim time
        lbl_time = QtWidgets.QLabel("Tiempo sim (s)")
        lbl_time.setFont(label_font)
        g_layout.addWidget(lbl_time, 3, 0)
        self.spin_time = QtWidgets.QDoubleSpinBox()
        self.spin_time.setRange(0.5, 60.0)
        self.spin_time.setSingleStep(0.5)
        self.spin_time.setValue(self.sim_time)
        self.spin_time.setFont(value_font)
        g_layout.addWidget(self.spin_time, 3, 1)

        # run / export buttons
        self.btn_run = QtWidgets.QPushButton("▶ Ejecutar simulación")
        self.btn_run.setFont(QtGui.QFont(FONT, 11, QtGui.QFont.Bold))
        self.btn_run.setStyleSheet(f"background:{PALETTE['accent2']}; color:black; padding:6px; border-radius:6px;")
        self.btn_run.clicked.connect(self.run_simulation)

        self.btn_export = QtWidgets.QPushButton("📤 Exportar resultados")
        self.btn_export.setFont(QtGui.QFont(FONT, 11))
        self.btn_export.setStyleSheet(f"background:{PALETTE['accent']}; color:white; padding:6px; border-radius:6px;")
        self.btn_export.clicked.connect(self.export_results)

        self.btn_toggle_motor = QtWidgets.QPushButton("💡 Animar motor (OFF)")
        self.btn_toggle_motor.setFont(QtGui.QFont(FONT, 10))
        self.btn_toggle_motor.setCheckable(True)
        self.btn_toggle_motor.toggled.connect(self._toggle_motor_animation)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addWidget(self.btn_run)
        btn_row.addWidget(self.btn_export)
        g_layout.addLayout(btn_row, 4, 0, 1, 2)
        g_layout.addWidget(self.btn_toggle_motor, 5, 0, 1, 2)

        # ---------- right: plots ----------
        # We'll create a gridspec: two main plots stacked + a small row of three mini axes for metrics
        self.fig_plots = Figure(figsize=(8,6), facecolor=PALETTE['bg'])
        gs = self.fig_plots.add_gridspec(4, 3)
        self.ax_step = self.fig_plots.add_subplot(gs[0:2, 0:3])   # main step response area
        self.ax_rpm = self.fig_plots.add_subplot(gs[2, 0:3])      # rpm response area
        self.ax_m1 = self.fig_plots.add_subplot(gs[3, 0])        # mini metric 1
        self.ax_m2 = self.fig_plots.add_subplot(gs[3, 1])        # mini metric 2
        self.ax_m3 = self.fig_plots.add_subplot(gs[3, 2])        # mini metric 3
        self.fig_plots.tight_layout(h_pad=2.0)

        # Canvas
        self.canvas_plots = FigureCanvas(self.fig_plots)
        right_col.addWidget(self.canvas_plots, 78)

        # set styles for axes
        for ax in (self.ax_step, self.ax_rpm, self.ax_m1, self.ax_m2, self.ax_m3):
            ax.set_facecolor(PALETTE['panel'])
            ax.tick_params(colors=PALETTE['text'])
            ax.grid(True, linestyle='--', alpha=0.35)

        self.ax_step.set_title('Respuesta al Escalón (normalizada)', color=PALETTE['text'])
        self.ax_rpm.set_title('Respuesta a Referencia (RPM, recortado a 0-100)', color=PALETTE['text'])

        self.ax_step.set_ylabel('Salida normalizada', color=PALETTE['text'])
        self.ax_rpm.set_ylabel('RPM', color=PALETTE['text'])
        self.ax_rpm.set_xlabel('Tiempo (s)', color=PALETTE['text'])

        # draw initial empty lines
        self.line_step, = self.ax_step.plot([], [], lw=2, color=PALETTE['accent'], label='Escalón')
        self.line_rpm, = self.ax_rpm.plot([], [], lw=2, color=PALETTE['accent2'], label='Salida (RPM)')
        self.line_target, = self.ax_rpm.plot([], [], lw=1, ls='--', color='#999999', label='Referencia')
        self.ax_rpm.set_ylim(0, 100)

        # ---------- metrics table ----------
        self.metrics_table = QtWidgets.QTableWidget(4, 3)
        self.metrics_table.setHorizontalHeaderLabels(["Métrica", "Escalón unitario", "Escalón a RPM"])
        self.metrics_table.verticalHeader().setVisible(False)
        self.metrics_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.metrics_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.metrics_table.setFocusPolicy(QtCore.Qt.NoFocus)
        self.metrics_table.setFixedHeight(190)
        metrics_names = ["Tiempo subida (s)", "Sobreimpulso (%)", "Tiempo asentamiento (s)", "Error estacionario"]
        header = self.metrics_table.horizontalHeader()
        header.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        header.setStyleSheet(f"QHeaderView::section {{ background: {PALETTE['card']}; color: {PALETTE['text']}; font-weight:bold; }}")

        for i, name in enumerate(metrics_names):
            item = QtWidgets.QTableWidgetItem(name)
            item.setFont(QtGui.QFont(FONT, 11))
            item.setForeground(QtGui.QColor(PALETTE['text']))
            self.metrics_table.setItem(i, 0, item)
            for c in (1, 2):
                it = QtWidgets.QTableWidgetItem("-")
                it.setTextAlignment(QtCore.Qt.AlignCenter)
                it.setForeground(QtGui.QColor(PALETTE['text']))
                it.setFont(QtGui.QFont(FONT, 11, QtGui.QFont.Bold))
                self.metrics_table.setItem(i, c, it)

        right_col.addWidget(self.metrics_table, 22)

        # initial draw
        self.canvas_motor.draw_idle()
        self.canvas_plots.draw_idle()

    # -----------------------
    # Motor visual elements
    # -----------------------
    def _setup_motor_visual_elements(self):
        ax = self.ax_motor
        ax.clear()
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)
        ax.axis('off')

        # main car/motor body
        self.body = patches.FancyBboxPatch((0.18, 0.32), 0.64, 0.28,
                                           boxstyle='round,pad=0.02',
                                           linewidth=2, edgecolor='#2b6ea3', facecolor='#2d7fb7', zorder=2)
        ax.add_patch(self.body)

        # hood overlay
        hood = patches.Polygon([[0.18,0.32],[0.18,0.45],[0.35,0.5],[0.45,0.5],[0.45,0.6],[0.82,0.6],[0.82,0.32]],
                               closed=True, facecolor='#357ab7', edgecolor=None, alpha=0.55, zorder=3)
        ax.add_patch(hood)

        # wheels
        self.wheel_l = patches.Circle((0.33,0.28), 0.06, facecolor='#111', edgecolor='#444', zorder=5)
        self.wheel_r = patches.Circle((0.67,0.28), 0.06, facecolor='#111', edgecolor='#444', zorder=5)
        ax.add_patch(self.wheel_l)
        ax.add_patch(self.wheel_r)

        # exhaust
        self.exhaust = patches.Rectangle((0.12,0.46), 0.06, 0.03, facecolor='#444', edgecolor='#222', zorder=2)
        ax.add_patch(self.exhaust)

        # humo particles (patches) pool
        self.humo = []
        for _ in range(18):
            c = patches.Circle((0.12 + random.uniform(0,0.06), 0.48 + random.uniform(0,0.04)),
                               radius=random.uniform(0.01,0.03), color='#bfbfbf', alpha=0.0, zorder=6)
            self.ax_motor.add_patch(c)
            self.humo.append(c)

        # spokes container
        self._spoke_lines = []

        self.status_text = self.ax_motor.text(0.5, 0.88, 'Estado: APAGADO', ha='center',
                                              color=PALETTE['danger'], fontsize=12, weight='bold', zorder=10)
        self.canvas_motor.draw_idle()

    # -----------------------
    # UI callbacks
    # -----------------------
    def _param_changed(self, *_):
        self.K = float(self.spin_K.value())
        self.tau = float(self.spin_tau.value())
        self.target_rpm = int(self.spin_ref.value())
        self.sim_time = float(self.spin_time.value())

    def run_simulation(self):
        # update params from UI
        self._param_changed()
        T = np.arange(0, self.sim_time + self.dt, self.dt)

        # build system
        sys = make_first_order_system(self.K, self.tau)

        # step response (unit step) and normalize for plotting (so axis ~0..1)
        t_step, y_step = simulate_step_response(sys, T)
        final_step = y_step[-1] if len(y_step) else 1.0
        if abs(final_step) > 1e-9:
            y_step_norm = y_step / final_step
        else:
            y_step_norm = y_step
        self.t_step = t_step
        self.y_step = y_step_norm

        # forced response to reach target_rpm: compute voltage needed and simulate
        voltage_needed = 0.0
        try:
            voltage_needed = float(self.target_rpm / self.K) if self.K != 0 else 0.0
        except Exception:
            voltage_needed = 0.0
        u = np.ones_like(T) * voltage_needed
        t_rpm, y_rpm = simulate_forced_response(sys, T, u)

        # clamp RPM's for display to 0..100 to preserve readability
        y_rpm_clamped = np.clip(y_rpm, 0.0, 100.0)
        self.t_rpm = t_rpm
        self.y_rpm = y_rpm_clamped

        # compute metrics (step normalized -> ref=1.0; rpm uses actual reference value)
        metrics_step = compute_metrics(self.t_step, self.y_step, 1.0)
        metrics_rpm = compute_metrics(self.t_rpm, self.y_rpm, float(self.target_rpm))
        self.latest_metrics = {"step": metrics_step, "rpm": metrics_rpm}

        # update plots and table
        self._update_plots()
        self._update_metrics_table(metrics_step, metrics_rpm)

        # keep also a time-series dataframe for export
        self.last_series = pd.DataFrame({
            't_step': self.t_step,
            'y_step_norm': self.y_step,
            't_rpm': self.t_rpm,
            'y_rpm': self.y_rpm
        })

        print('[INFO] Simulación completada.')

    def _update_plots(self):
        # step plot
        self.ax_step.clear()
        self.ax_step.set_facecolor(PALETTE['panel'])
        self.ax_step.plot(self.t_step, self.y_step, lw=2, color=PALETTE['accent'])
        self.ax_step.set_ylabel('Salida (normalizada)', color=PALETTE['text'])
        self.ax_step.set_ylim(0, 1.2)
        self.ax_step.tick_params(colors=PALETTE['text'])
        self.ax_step.grid(True, linestyle='--', alpha=0.35)

        # rpm plot
        self.ax_rpm.clear()
        self.ax_rpm.set_facecolor(PALETTE['panel'])
        self.ax_rpm.plot(self.t_rpm, self.y_rpm, lw=2, color=PALETTE['accent2'], label='Salida (RPM)')
        if len(self.t_rpm):
            self.ax_rpm.plot([self.t_rpm[0], self.t_rpm[-1]], [self.target_rpm, self.target_rpm],
                             lw=1.2, ls='--', color='#999999', label=f'Referencia {self.target_rpm} RPM')
        self.ax_rpm.set_ylabel('RPM', color=PALETTE['text'])
        self.ax_rpm.set_ylim(0, 100)
        self.ax_rpm.tick_params(colors=PALETTE['text'])
        self.ax_rpm.grid(True, linestyle='--', alpha=0.35)
        self.ax_rpm.legend(facecolor=PALETTE['panel'], edgecolor='#222', labelcolor=PALETTE['text'])

        # mini metrics (bars): compare step vs rpm metrics
        self.ax_m1.clear(); self.ax_m2.clear(); self.ax_m3.clear()
        # prepare values
        if self.latest_metrics:
            ms = self.latest_metrics['step']; mr = self.latest_metrics['rpm']
            # tiempo subida
            vals_tr = [ms.get('t_rise', np.nan) or np.nan, mr.get('t_rise', np.nan) or np.nan]
            # sobreimpulso
            vals_mp = [ms.get('overshoot', np.nan) or np.nan, mr.get('overshoot', np.nan) or np.nan]
            # error estacionario
            vals_ess = [ms.get('ess', np.nan) or np.nan, mr.get('ess', np.nan) or np.nan]
        else:
            vals_tr = vals_mp = vals_ess = [np.nan, np.nan]

        labels = ['Escalón', 'RPM']
        x = np.arange(len(labels))
        self.ax_m1.bar(x, vals_tr, color=[PALETTE['accent'], PALETTE['accent2']])
        self.ax_m1.set_title('Tiempo subida (s)', color=PALETTE['text'], fontsize=9)
        self.ax_m2.bar(x, vals_mp, color=[PALETTE['accent'], PALETTE['accent2']])
        self.ax_m2.set_title('Sobreimpulso (%)', color=PALETTE['text'], fontsize=9)
        self.ax_m3.bar(x, vals_ess, color=[PALETTE['accent'], PALETTE['accent2']])
        self.ax_m3.set_title('Error estacionario', color=PALETTE['text'], fontsize=9)

        # draw
        self.canvas_plots.draw_idle()

    def _update_metrics_table(self, metrics_step, metrics_rpm):
        rows = [("Tiempo subida (s)", "t_rise"), ("Sobreimpulso (%)", "overshoot"),
                ("Tiempo asentamiento (s)", "t_settle"), ("Error estacionario", "ess")]
        for i, (label, key) in enumerate(rows):
            val_step = metrics_step.get(key, np.nan)
            val_rpm = metrics_rpm.get(key, np.nan)
            it1 = QtWidgets.QTableWidgetItem(f"{val_step:.3f}" if not (val_step is None or np.isnan(val_step)) else "N/A")
            it1.setTextAlignment(QtCore.Qt.AlignCenter)
            it1.setForeground(QtGui.QColor(PALETTE['text']))
            it1.setFont(QtGui.QFont(FONT, 11, QtGui.QFont.Bold))
            self.metrics_table.setItem(i, 1, it1)

            it2 = QtWidgets.QTableWidgetItem(f"{val_rpm:.3f}" if not (val_rpm is None or np.isnan(val_rpm)) else "N/A")
            it2.setTextAlignment(QtCore.Qt.AlignCenter)
            it2.setForeground(QtGui.QColor(PALETTE['text']))
            it2.setFont(QtGui.QFont(FONT, 11, QtGui.QFont.Bold))
            self.metrics_table.setItem(i, 2, it2)

    # -----------------------
    # Motor animation & smoke
    # -----------------------
    def _toggle_motor_animation(self, checked):
        self.motor_anim_on = bool(checked)
        self.btn_toggle_motor.setText("💡 Animar motor (ON)" if self.motor_anim_on else "💡 Animar motor (OFF)")

    def _animate_motor(self):
        # animate wheel spokes and smoke particles
        if self.motor_anim_on and len(self.y_rpm) > 0:
            rpm_est = float(self.y_rpm[-1]) if len(self.y_rpm) else 0.0
        else:
            rpm_est = 0.0

        # clamp for rotation speed
        rpm_clamped = max(0.0, min(100.0, rpm_est))
        angle_inc = rpm_clamped * 0.5 * 0.02  # tuned factor

        # remove old spoke lines
        for ln in list(self._spoke_lines):
            try:
                ln.remove()
            except Exception:
                pass
        self._spoke_lines.clear()

        # draw new spokes
        centers = [(0.33, 0.28), (0.67, 0.28)]
        if not hasattr(self, '_wheel_angle'):
            self._wheel_angle = 0.0
        self._wheel_angle = (self._wheel_angle + angle_inc) % 360
        for cx, cy in centers:
            for s in range(6):
                angle = math.radians(self._wheel_angle + s * (360 / 6))
                x_end = cx + 0.045 * math.cos(angle)
                y_end = cy + 0.045 * math.sin(angle)
                ln, = self.ax_motor.plot([cx, x_end], [cy, y_end], color="#aaaaaa", linewidth=1.0, zorder=7)
                self._spoke_lines.append(ln)

        # update smoke particles intensity from latest error
        if self.latest_metrics:
            err = float(self.latest_metrics.get('rpm', {}).get('ess', 0.0))
        else:
            err = 0.0
        intensity = float(min(1.0, 3.0 * err))

        if self.motor_anim_on:
            for patch in self.humo:
                x, y = patch.center
                # birth
                if patch.get_alpha() <= 0.01 and random.random() < 0.25 * (0.3 + intensity):
                    patch.center = (0.12 + random.uniform(0.0, 0.06), 0.48 + random.uniform(0.0, 0.02))
                    patch.radius = random.uniform(0.01, 0.03) * (1.0 + intensity)
                    patch.set_alpha(0.25 + 0.45 * intensity)
                else:
                    nx = x + random.uniform(-0.002, 0.002)
                    ny = y + 0.008 + 0.015 * intensity
                    nr = patch.radius * (1.0 + 0.003 * intensity)
                    if ny > 0.86:
                        nx = 0.12 + random.uniform(0.0, 0.06)
                        ny = 0.48 + random.uniform(0.0, 0.02)
                        nr = random.uniform(0.01, 0.03)
                        patch.set_alpha(0.0)
                    patch.center = (nx, ny)
                    patch.radius = nr
                    patch.set_alpha(max(0.0, patch.get_alpha() - 0.008))
        else:
            for patch in self.humo:
                patch.set_alpha(max(0.0, patch.get_alpha() - 0.05))

        # status color change based on intensity
        if intensity > 0.15:
            self.body.set_facecolor(PALETTE['accent2'])
            self.status_text.set_text('Estado: ENCENDIDO (PERTURBACIÓN)')
            self.status_text.set_color(PALETTE['accent2'])
        elif self.motor_anim_on:
            self.body.set_facecolor('#00C49A')
            self.status_text.set_text('Estado: ENCENDIDO')
            self.status_text.set_color(PALETTE['accent2'])
        else:
            self.body.set_facecolor('#2d7fb7')
            self.status_text.set_text('Estado: APAGADO')
            self.status_text.set_color(PALETTE['danger'])

        self.canvas_motor.draw_idle()

    # -----------------------
    # Export results: PNG + CSV
    # -----------------------
    def export_results(self):
        out_dir = os.path.join('outputs', 'motor_malo_model')
        os.makedirs(out_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        csv_path = None
        if hasattr(self, 'latest_metrics') and self.latest_metrics is not None:
            csv_path = os.path.join(out_dir, f'metrics_{timestamp}.csv')
            rows = []
            step = self.latest_metrics.get('step', {})
            rpm = self.latest_metrics.get('rpm', {})
            keys = ['t_rise', 'overshoot', 't_settle', 'ess']
            labels = ['Tiempo subida (s)', 'Sobreimpulso (%)', 'Tiempo asentamiento (s)', 'Error estacionario']
            for lab, k in zip(labels, keys):
                rows.append({'Métrica': lab, 'Escalón unitario': step.get(k, ''), 'Escalón a RPM': rpm.get(k, '')})
            df = pd.DataFrame(rows)
            df.to_csv(csv_path, index=False)

        # save time-series if available
        series_path = None
        if hasattr(self, 'last_series'):
            series_path = os.path.join(out_dir, f'series_{timestamp}.csv')
            self.last_series.to_csv(series_path, index=False)

        png_path = os.path.join(out_dir, f'plots_{timestamp}.png')
        try:
            self.fig_plots.savefig(png_path, dpi=220, facecolor=self.fig_plots.get_facecolor())
        except Exception:
            png_path = None

        QtWidgets.QMessageBox.information(self, 'Exportación completada',
                                          f'Guardados:\n{csv_path if csv_path else "No metrics"}\n{series_path if series_path else "No series"}\n{png_path if png_path else "No plot"}')


# -----------------------
# Run app
# -----------------------

def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = MotorMaloModelGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
