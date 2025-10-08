# motor_malo_pid.py
"""
Motor Malo - Controlador PID compensado (ISAI)
Implementa un controlador PID sobre el modelo de primer orden.
"""
from pid_controller import PIDController
from motor_malo_model import motor_malo_step
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
# PID Defaults (P/PI/PID: se empieza con P puro para facilitar el análisis)
DEFAULT_KP = 1.0
DEFAULT_KI = 0.0
DEFAULT_KD = 0.0
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
        # TransferFunction requiere el numerador y denominador como listas de coeficientes
        return signal.TransferFunction([K], [tau, 1])


def make_pid_controller(Kp, Ki, Kd):
    """Return TransferFunction C(s) = Kp + Ki/s + Kd*s."""
    if USE_CONTROL:
        # C(s) = (Kd*s^2 + Kp*s + Ki) / s
        return ctl.TransferFunction([Kd, Kp, Ki], [1, 0])
    else:
        # C(s) = (Kd*s^2 + Kp*s + Ki) / s
        return signal.TransferFunction([Kd, Kp, Ki], [1, 0])


def simulate_step_response(sys, T):
    """Simulate unit step response on time vector T."""
    if USE_CONTROL:
        t, y = ctl.step_response(sys, T=T)
    else:
        t, y = signal.step(sys, T=T)
    return np.asarray(t), np.asarray(y)


# NOTA: En la simulación PID, el sistema ya está en lazo cerrado,
# por lo que la "forced_response" se reemplazará por la simulación de escalón
# sobre el sistema CLTF.

def compute_metrics(t, y, ref):
    """Compute rise time (10-90%), overshoot (%), settling time (2%), steady-state error.
    Returns dict with float values (NaN if not computable).
    """
    # [Lógica de compute_metrics SIN CAMBIOS, se mantiene la de tu código original]
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

def test_pid_stability():
    K = DEFAULT_K
    tau = DEFAULT_TAU
    Ts = 0.02
    ref = DEFAULT_TARGET_RPM
    pid = PIDController(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, Ts)
    rpm = 0
    rpm_hist = []
    u_hist = []
    for i in range(int(DEFAULT_SIM_TIME / Ts)):
        t = i * Ts
        # Falla simulada entre 4 y 6 segundos
        falla = 4 < t < 6
        ruido_std = 5.0 if not falla else 25.0
        rpm = motor_malo_step(rpm, ref, K, tau, Ts, ruido_std=ruido_std, falla=falla)
        u = pid.calcular_salida(ref, rpm)
        rpm_hist.append(rpm)
        u_hist.append(u)
        # Checa saturación
        if abs(u) >= pid.limite_u:
            print(f"Saturación del PID en t={t:.2f}s")
        # Checa estabilidad
        if abs(rpm) > 2*ref:
            print(f"Inestabilidad detectada en t={t:.2f}s")
    return rpm_hist, u_hist

# -----------------------
# Main GUI application
# -----------------------
class MotorMaloPIDGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Malo - PID Compensado (ISAI)")
        self.setMinimumSize(1260, 820)
        self.setStyleSheet(f"background-color: {PALETTE['bg']}; color: {PALETTE['text']};")

        # state
        self.K = DEFAULT_K
        self.tau = DEFAULT_TAU
        # NUEVOS PARÁMETROS PID
        self.Kp = DEFAULT_KP
        self.Ki = DEFAULT_KI
        self.Kd = DEFAULT_KD

        self.target_rpm = DEFAULT_TARGET_RPM
        self.sim_time = DEFAULT_SIM_TIME
        self.dt = 0.01
        self.motor_anim_on = False

        # histories
        # Ahora el historial RPM/Step es el mismo, ya que simulamos el lazo cerrado
        self.t_cl = np.array([])  # CL: Closed Loop
        self.y_cl = np.array([])

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

        # [CÓDIGO DE VISUALIZACIÓN DEL MOTOR SIN CAMBIOS]
        # ---------- motor visual (matplotlib canvas) ----------
        self.fig_motor = Figure(figsize=(5, 3), facecolor=PALETTE["bg"])
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

        # K (Fila 0)
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

        # tau (Fila 1)
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

        # ------------------- NUEVOS CONTROLES PID -------------------
        # Kp (Fila 2)
        lbl_kp = QtWidgets.QLabel("Kp (Proporcional)")
        lbl_kp.setFont(label_font)
        g_layout.addWidget(lbl_kp, 2, 0)
        self.spin_Kp = QtWidgets.QDoubleSpinBox()
        self.spin_Kp.setRange(0.0, 1000.0)
        self.spin_Kp.setSingleStep(0.1)
        self.spin_Kp.setValue(self.Kp)
        self.spin_Kp.setFont(value_font)
        self.spin_Kp.valueChanged.connect(self._param_changed)
        g_layout.addWidget(self.spin_Kp, 2, 1)

        # Ki (Fila 3)
        lbl_ki = QtWidgets.QLabel("Ki (Integral)")
        lbl_ki.setFont(label_font)
        g_layout.addWidget(lbl_ki, 3, 0)
        self.spin_Ki = QtWidgets.QDoubleSpinBox()
        self.spin_Ki.setRange(0.0, 1000.0)
        self.spin_Ki.setSingleStep(0.1)
        self.spin_Ki.setValue(self.Ki)
        self.spin_Ki.setFont(value_font)
        self.spin_Ki.valueChanged.connect(self._param_changed)
        g_layout.addWidget(self.spin_Ki, 3, 1)

        # Kd (Fila 4)
        lbl_kd = QtWidgets.QLabel("Kd (Derivativo)")
        lbl_kd.setFont(label_font)
        g_layout.addWidget(lbl_kd, 4, 0)
        self.spin_Kd = QtWidgets.QDoubleSpinBox()
        self.spin_Kd.setRange(0.0, 1000.0)
        self.spin_Kd.setSingleStep(0.1)
        self.spin_Kd.setValue(self.Kd)
        self.spin_Kd.setFont(value_font)
        self.spin_Kd.valueChanged.connect(self._param_changed)
        g_layout.addWidget(self.spin_Kd, 4, 1)
        # ------------------- FIN NUEVOS CONTROLES PID -------------------

        # target rpm (Fila 5)
        lbl_ref = QtWidgets.QLabel("Referencia RPM (0-100)")
        lbl_ref.setFont(label_font)
        g_layout.addWidget(lbl_ref, 5, 0)
        self.spin_ref = QtWidgets.QSpinBox()
        self.spin_ref.setRange(0, 100)
        self.spin_ref.setSingleStep(1)
        self.spin_ref.setValue(self.target_rpm)
        self.spin_ref.setFont(value_font)
        self.spin_ref.valueChanged.connect(self._param_changed)
        g_layout.addWidget(self.spin_ref, 5, 1)

        # sim time (Fila 6)
        lbl_time = QtWidgets.QLabel("Tiempo sim (s)")
        lbl_time.setFont(label_font)
        g_layout.addWidget(lbl_time, 6, 0)
        self.spin_time = QtWidgets.QDoubleSpinBox()
        self.spin_time.setRange(0.5, 60.0)
        self.spin_time.setSingleStep(0.5)
        self.spin_time.setValue(self.sim_time)
        self.spin_time.setFont(value_font)
        g_layout.addWidget(self.spin_time, 6, 1)

        # run / export buttons (Filas 7 y 8)
        self.btn_run = QtWidgets.QPushButton("▶ Ejecutar simulación PID")
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
        g_layout.addLayout(btn_row, 7, 0, 1, 2)
        g_layout.addWidget(self.btn_toggle_motor, 8, 0, 1, 2)

        # [CÓDIGO DE PLOTEO Y MÉTRICAS SIN CAMBIOS MAYORES]
        # ---------- right: plots ----------
        self.fig_plots = Figure(figsize=(8, 6), facecolor=PALETTE['bg'])
        gs = self.fig_plots.add_gridspec(4, 3)
        # Solo necesitamos un plot principal para lazo cerrado, mantenemos la estructura
        self.ax_cl_response = self.fig_plots.add_subplot(gs[0:3, 0:3])  # Lazo cerrado
        self.ax_m1 = self.fig_plots.add_subplot(gs[3, 0])  # mini metric 1
        self.ax_m2 = self.fig_plots.add_subplot(gs[3, 1])  # mini metric 2
        self.ax_m3 = self.fig_plots.add_subplot(gs[3, 2])  # mini metric 3
        self.fig_plots.tight_layout(h_pad=2.0)

        # Canvas
        self.canvas_plots = FigureCanvas(self.fig_plots)
        right_col.addWidget(self.canvas_plots, 78)

        # set styles for axes
        for ax in (self.ax_cl_response, self.ax_m1, self.ax_m2, self.ax_m3):
            ax.set_facecolor(PALETTE['panel'])
            ax.tick_params(colors=PALETTE['text'])
            ax.grid(True, linestyle='--', alpha=0.35)

        self.ax_cl_response.set_title('Respuesta de Lazo Cerrado con PID (RPM, 0-100)', color=PALETTE['text'])
        self.ax_cl_response.set_ylabel('RPM', color=PALETTE['text'])
        self.ax_cl_response.set_xlabel('Tiempo (s)', color=PALETTE['text'])

        # draw initial empty lines
        self.line_cl, = self.ax_cl_response.plot([], [], lw=2, color=PALETTE['accent2'], label='Salida (RPM)')
        self.line_target, = self.ax_cl_response.plot([], [], lw=1, ls='--', color='#999999', label='Referencia')
        self.ax_cl_response.set_ylim(0, 100)
        self.ax_cl_response.legend(facecolor=PALETTE['panel'], edgecolor='#222', labelcolor=PALETTE['text'])

        # ---------- metrics table (adaptada a solo lazo cerrado) ----------
        self.metrics_table = QtWidgets.QTableWidget(4, 2)
        self.metrics_table.setHorizontalHeaderLabels(["Métrica", "Valor (PID)"])
        self.metrics_table.verticalHeader().setVisible(False)
        self.metrics_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.metrics_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.metrics_table.setFocusPolicy(QtCore.Qt.NoFocus)
        self.metrics_table.setFixedHeight(190)
        metrics_names = ["Tiempo subida (s)", "Sobreimpulso (%)", "Tiempo asentamiento (s)", "Error estacionario"]
        header = self.metrics_table.horizontalHeader()
        header.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        header.setStyleSheet(
            f"QHeaderView::section {{ background: {PALETTE['card']}; color: {PALETTE['text']}; font-weight:bold; }}")

        for i, name in enumerate(metrics_names):
            item = QtWidgets.QTableWidgetItem(name)
            item.setFont(QtGui.QFont(FONT, 11))
            item.setForeground(QtGui.QColor(PALETTE['text']))
            self.metrics_table.setItem(i, 0, item)
            it = QtWidgets.QTableWidgetItem("-")
            it.setTextAlignment(QtCore.Qt.AlignCenter)
            it.setForeground(QtGui.QColor(PALETTE['text']))
            it.setFont(QtGui.QFont(FONT, 11, QtGui.QFont.Bold))
            self.metrics_table.setItem(i, 1, it)

        right_col.addWidget(self.metrics_table, 22)

        # initial draw
        self.canvas_motor.draw_idle()
        self.canvas_plots.draw_idle()

    # [CÓDIGO DE VISUALIZACIÓN DEL MOTOR SIN CAMBIOS]
    def _setup_motor_visual_elements(self):
        # ... (Mantener el código de _setup_motor_visual_elements)
        ax = self.ax_motor
        ax.clear()
        ax.set_xlim(0, 1);
        ax.set_ylim(0, 1)
        ax.axis('off')

        # main car/motor body
        self.body = patches.FancyBboxPatch((0.18, 0.32), 0.64, 0.28,
                                           boxstyle='round,pad=0.02',
                                           linewidth=2, edgecolor='#2b6ea3', facecolor='#2d7fb7', zorder=2)
        ax.add_patch(self.body)

        # hood overlay
        hood = patches.Polygon(
            [[0.18, 0.32], [0.18, 0.45], [0.35, 0.5], [0.45, 0.5], [0.45, 0.6], [0.82, 0.6], [0.82, 0.32]],
            closed=True, facecolor='#357ab7', edgecolor=None, alpha=0.55, zorder=3)
        ax.add_patch(hood)

        # wheels
        self.wheel_l = patches.Circle((0.33, 0.28), 0.06, facecolor='#111', edgecolor='#444', zorder=5)
        self.wheel_r = patches.Circle((0.67, 0.28), 0.06, facecolor='#111', edgecolor='#444', zorder=5)
        ax.add_patch(self.wheel_l)
        ax.add_patch(self.wheel_r)

        # exhaust
        self.exhaust = patches.Rectangle((0.12, 0.46), 0.06, 0.03, facecolor='#444', edgecolor='#222', zorder=2)
        ax.add_patch(self.exhaust)

        # humo particles (patches) pool
        self.humo = []
        for _ in range(18):
            c = patches.Circle((0.12 + random.uniform(0, 0.06), 0.48 + random.uniform(0, 0.04)),
                               radius=random.uniform(0.01, 0.03), color='#bfbfbf', alpha=0.0, zorder=6)
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
        # Leer nuevos parámetros PID
        self.Kp = float(self.spin_Kp.value())
        self.Ki = float(self.spin_Ki.value())
        self.Kd = float(self.spin_Kd.value())

        self.target_rpm = int(self.spin_ref.value())
        self.sim_time = float(self.spin_time.value())

    def run_simulation(self):
        # update params from UI
        self._param_changed()
        T = np.arange(0, self.sim_time + self.dt, self.dt)

        # 1. Modelo del Motor (Planta)
        G_motor = make_first_order_system(self.K, self.tau)

        # 2. Controlador PID
        C_pid = make_pid_controller(self.Kp, self.Ki, self.Kd)

        # 3. Lazo Cerrado (Closed-Loop Transfer Function, CLTF)
        # G_cl = C*G / (1 + C*G)
        if USE_CONTROL:
            G_cl = ctl.feedback(C_pid * G_motor, 1)  # realimentación unitaria negativa
        else:
            # Multiplicar sistemas: CG = C * G
            num_cg = np.convolve(C_pid.num[0], G_motor.num[0])
            den_cg = np.convolve(C_pid.den[0], G_motor.den[0])

            # 1 + CG = Den(CG) + Num(CG) / Den(CG)
            den_cl = num_cg + den_cg

            # CLTF = Num(CG) / (Num(CG) + Den(CG))
            G_cl = signal.TransferFunction(num_cg, den_cl)

            # Nota: para la librería scipy.signal, la realimentación requiere un manejo 
            # más explícito de los polinomios de transferencia.
            # La implementación simple de LTI con signal.lsim o signal.step es más compleja
            # para el lazo cerrado, por eso se prefiere control.

        # 4. Simulación de la respuesta al escalón con la referencia (target_rpm)
        # Dado que el sistema ya es el lazo cerrado G_cl, la entrada es la referencia.
        # Escalar el sistema para que el escalón unitario produzca 'target_rpm'
        # o simular el escalón unitario y luego escalarlo. Usaremos el escalón unitario (ref=1)
        # para simular la respuesta y luego multiplicaremos por la referencia.

        # Simulación de respuesta al escalón
        t_cl, y_cl_unit = simulate_step_response(G_cl, T)

        # Escalamos la respuesta a la referencia RPM
        y_cl_rpm = y_cl_unit * self.target_rpm

        # clamp RPM's for display to 0..100 to preserve readability
        y_cl_clamped = np.clip(y_cl_rpm, 0.0, 100.0)
        self.t_cl = t_cl
        self.y_cl = y_cl_clamped

        # compute metrics (usando la referencia target_rpm)
        metrics_cl = compute_metrics(self.t_cl, self.y_cl, float(self.target_rpm))
        self.latest_metrics = {"cl": metrics_cl}

        # update plots and table
        self._update_plots()
        self._update_metrics_table(metrics_cl)

        # keep also a time-series dataframe for export
        self.last_series = pd.DataFrame({
            't_closed_loop': self.t_cl,
            'y_closed_loop_rpm': self.y_cl
        })

        print(f'[INFO] Simulación PID completada. Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}')

    def _update_plots(self):
        # Gráfica de respuesta de Lazo Cerrado
        self.ax_cl_response.clear()
        self.ax_cl_response.set_facecolor(PALETTE['panel'])
        self.ax_cl_response.plot(self.t_cl, self.y_cl, lw=2, color=PALETTE['accent2'], label='Salida (RPM)')
        if len(self.t_cl):
            self.ax_cl_response.plot([self.t_cl[0], self.t_cl[-1]], [self.target_rpm, self.target_rpm],
                                     lw=1.2, ls='--', color='#999999', label=f'Referencia {self.target_rpm} RPM')
        self.ax_cl_response.set_ylabel('RPM', color=PALETTE['text'])
        self.ax_cl_response.set_xlabel('Tiempo (s)', color=PALETTE['text'])
        self.ax_cl_response.set_ylim(0, 100)
        self.ax_cl_response.tick_params(colors=PALETTE['text'])
        self.ax_cl_response.grid(True, linestyle='--', alpha=0.35)
        self.ax_cl_response.legend(facecolor=PALETTE['panel'], edgecolor='#222', labelcolor=PALETTE['text'])

        # mini metrics (bars): solo comparamos la métrica CL
        self.ax_m1.clear();
        self.ax_m2.clear();
        self.ax_m3.clear()

        # Valores de ejemplo para las mini-gráficas de barras
        if self.latest_metrics:
            mr = self.latest_metrics['cl']
            vals_tr = [mr.get('t_rise', np.nan) or np.nan, 0]  # el 0 es solo para que la barra se vea como una sola
            vals_mp = [mr.get('overshoot', np.nan) or np.nan, 0]
            vals_ess = [mr.get('ess', np.nan) or np.nan, 0]
        else:
            vals_tr = vals_mp = vals_ess = [np.nan, np.nan]

        labels = ['PID', '']
        x = np.arange(len(labels))

        # Tiempo subida
        self.ax_m1.bar(x, vals_tr, color=[PALETTE['accent2'], '#0d1117'])
        self.ax_m1.set_title('Tiempo subida (s)', color=PALETTE['text'], fontsize=9)
        self.ax_m1.set_xticks(x, labels)

        # Sobreimpulso
        self.ax_m2.bar(x, vals_mp, color=[PALETTE['accent2'], '#0d1117'])
        self.ax_m2.set_title('Sobreimpulso (%)', color=PALETTE['text'], fontsize=9)
        self.ax_m2.set_xticks(x, labels)

        # Error estacionario
        self.ax_m3.bar(x, vals_ess, color=[PALETTE['accent2'], '#0d1117'])
        self.ax_m3.set_title('Error estacionario', color=PALETTE['text'], fontsize=9)
        self.ax_m3.set_xticks(x, labels)

        # draw
        self.canvas_plots.draw_idle()

    def _update_metrics_table(self, metrics_cl):
        rows = [("Tiempo subida (s)", "t_rise"), ("Sobreimpulso (%)", "overshoot"),
                ("Tiempo asentamiento (s)", "t_settle"), ("Error estacionario", "ess")]
        for i, (label, key) in enumerate(rows):
            val_cl = metrics_cl.get(key, np.nan)
            it1 = QtWidgets.QTableWidgetItem(f"{val_cl:.3f}" if not (val_cl is None or np.isnan(val_cl)) else "N/A")
            it1.setTextAlignment(QtCore.Qt.AlignCenter)
            it1.setForeground(QtGui.QColor(PALETTE['text']))
            it1.setFont(QtGui.QFont(FONT, 11, QtGui.QFont.Bold))
            self.metrics_table.setItem(i, 1, it1)

    # [CÓDIGO DE ANIMACIÓN Y EXPORTACIÓN CON ADAPTACIONES MÍNIMAS]
    def _toggle_motor_animation(self, checked):
        # ... (Mantener el código de _toggle_motor_animation)
        self.motor_anim_on = bool(checked)
        self.btn_toggle_motor.setText("💡 Animar motor (ON)" if self.motor_anim_on else "💡 Animar motor (OFF)")

    def _animate_motor(self):
        # ... (Mantener el código de _animate_motor, usando self.y_cl para RPM)
        if self.motor_anim_on and len(self.y_cl) > 0:
            rpm_est = float(self.y_cl[-1]) if len(self.y_cl) else 0.0
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
            err = float(self.latest_metrics.get('cl', {}).get('ess', 0.0))
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

    def export_results(self):
        # ... (Adaptar la exportación para un solo conjunto de métricas CL)
        out_dir = os.path.join('outputs', 'motor_malo_pid')
        os.makedirs(out_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        csv_path = None
        if hasattr(self, 'latest_metrics') and self.latest_metrics is not None:
            csv_path = os.path.join(out_dir, f'metrics_{timestamp}.csv')
            rows = []
            cl = self.latest_metrics.get('cl', {})
            keys = ['t_rise', 'overshoot', 't_settle', 'ess']
            labels = ['Tiempo subida (s)', 'Sobreimpulso (%)', 'Tiempo asentamiento (s)', 'Error estacionario']
            for lab, k in zip(labels, keys):
                rows.append({'Métrica': lab, 'Valor (PID)': cl.get(k, '')})
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
    gui = MotorMaloPIDGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    # Asegurarse de que 'control' esté instalado
    if not USE_CONTROL:
        print(
            "ADVERTENCIA: La librería 'control' no está instalada. La simulación PID en lazo cerrado será muy limitada o fallará con scipy.signal.")
    main()