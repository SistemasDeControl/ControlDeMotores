"""
motor_malo_parametric_test.py
Semana 2 - Integrante 3 (ISAI)
Interfaz de pruebas paramétricas del "motor malo" con perturbaciones aleatorias.

Genera:
 - ./outputs/motor_malo_parametric_test/resultados_parametricos.csv
 - ./outputs/motor_malo_parametric_test/grafico_resumen.png
"""
from motor_malo_model import motor_malo_step
import os
import sys
import random
import numpy as np
import pandas as pd
from datetime import datetime
import matplotlib.pyplot as plt

from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

try:
    import control as ctl
    USE_CONTROL = True
except Exception:
    from scipy import signal
    USE_CONTROL = False


# ======= CONFIGURACIÓN VISUAL =======
plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.facecolor": "#0d1117",
    "figure.facecolor": "#0d1117",
    "axes.labelcolor": "white",
    "xtick.color": "white",
    "ytick.color": "white",
    "text.color": "white",
})

# ======= MODELO BASE =======
BASE_K = 0.6
BASE_TAU = 1.2
TARGET_RPM = 100
TIME_VECTOR = np.linspace(0, 8, 400)


def make_first_order_system(K: float, tau: float):
    if USE_CONTROL:
        return ctl.TransferFunction([K], [tau, 1])
    else:
        return signal.TransferFunction([K], [tau, 1])


def simulate_forced_response(sys, T, u):
    if USE_CONTROL:
        t_out, y_out = ctl.forced_response(sys, T=T, U=u)
    else:
        t_out, y_out, _ = signal.lsim(sys, U=u, T=T)
    return t_out, y_out


# ======= INTERFAZ PRINCIPAL =======
class MotorMaloParametricTestGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Test Paramétrico - Motor Malo (ISAI)")
        self.setGeometry(200, 100, 1300, 700)
        self.setStyleSheet("background-color: #0d1117; color: white;")

        # Variables
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.run_parametric_step)
        self.running = False
        self.test_index = 0
        self.data_records = []

        # --- Parámetros de perturbación
        self.perturb_K = 0.25
        self.perturb_tau = 0.3
        self.noise_amp = 5.0

        # --- UI Layout principal
        main_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QHBoxLayout(main_widget)
        self.setCentralWidget(main_widget)

        # ====== Panel Izquierdo (controles)
        control_layout = QtWidgets.QVBoxLayout()
        control_layout.setSpacing(15)

        self.btn_start = QtWidgets.QPushButton("▶ Iniciar Test")
        self.btn_start.clicked.connect(self.start_test)
        self.btn_start.setStyleSheet("background-color: #00C49A; color: black; font-weight: bold; font-size: 14px;")

        self.btn_stop = QtWidgets.QPushButton("⏹ Detener Test")
        self.btn_stop.clicked.connect(self.stop_test)
        self.btn_stop.setStyleSheet("background-color: #FF6B6B; color: white; font-weight: bold; font-size: 14px;")

        self.btn_export = QtWidgets.QPushButton("💾 Exportar Resultados")
        self.btn_export.clicked.connect(self.export_results)
        self.btn_export.setStyleSheet("background-color: #0078D7; color: white; font-weight: bold; font-size: 14px;")

        control_layout.addWidget(QtWidgets.QLabel("⚙️ Parámetros base:"))
        control_layout.addWidget(QtWidgets.QLabel(f"K base = {BASE_K}"))
        control_layout.addWidget(QtWidgets.QLabel(f"τ base = {BASE_TAU}"))
        control_layout.addSpacing(10)
        control_layout.addWidget(QtWidgets.QLabel("Perturbación K ±"))
        self.slider_K = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_K.setMinimum(1)
        self.slider_K.setMaximum(50)
        self.slider_K.setValue(int(self.perturb_K * 100))
        self.slider_K.valueChanged.connect(self.update_sliders)
        control_layout.addWidget(self.slider_K)

        control_layout.addWidget(QtWidgets.QLabel("Perturbación τ ±"))
        self.slider_tau = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_tau.setMinimum(1)
        self.slider_tau.setMaximum(50)
        self.slider_tau.setValue(int(self.perturb_tau * 100))
        self.slider_tau.valueChanged.connect(self.update_sliders)
        control_layout.addWidget(self.slider_tau)

        control_layout.addWidget(QtWidgets.QLabel("Ruido de entrada"))
        self.slider_noise = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_noise.setMinimum(1)
        self.slider_noise.setMaximum(100)
        self.slider_noise.setValue(int(self.noise_amp))
        self.slider_noise.valueChanged.connect(self.update_sliders)
        control_layout.addWidget(self.slider_noise)

        control_layout.addWidget(self.btn_start)
        control_layout.addWidget(self.btn_stop)
        control_layout.addWidget(self.btn_export)
        control_layout.addStretch(1)

        # ====== Panel Central (gráficas)
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 7))
        self.fig.subplots_adjust(hspace=0.4)
        self.ax1.set_title("Respuesta en RPM bajo perturbaciones", color="white")
        self.ax1.set_ylim(0, TARGET_RPM + 10)
        self.ax1.grid(True, color="#333")

        self.ax2.set_title("Evolución de parámetros K y τ", color="white")
        self.ax2.grid(True, color="#333")

        self.canvas = FigureCanvas(self.fig)
        self.curve_lines = []
        self.k_values = []
        self.tau_values = []

        # ====== Panel Derecho (tabla)
        self.table = QtWidgets.QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["Test", "K", "τ", "Overshoot (%)"])
        self.table.horizontalHeader().setStyleSheet("color: white; background-color: #0078D7; font-weight: bold;")
        self.table.setStyleSheet("QTableWidget {color: white; gridline-color: gray;}")
        self.table.setColumnWidth(0, 70)
        self.table.setColumnWidth(1, 100)
        self.table.setColumnWidth(2, 100)
        self.table.setColumnWidth(3, 130)

        # --- Armar layout final
        main_layout.addLayout(control_layout, 1)
        main_layout.addWidget(self.canvas, 3)
        main_layout.addWidget(self.table, 1)

    # ====== Métodos funcionales ======
    def update_sliders(self):
        self.perturb_K = self.slider_K.value() / 100.0
        self.perturb_tau = self.slider_tau.value() / 100.0
        self.noise_amp = self.slider_noise.value()

    def start_test(self):
        if not self.running:
            self.running = True
            self.timer.start(1000)
            self.test_index = 0
            print("[INFO] Test paramétrico iniciado.")

    def stop_test(self):
        self.running = False
        self.timer.stop()
        print("[INFO] Test paramétrico detenido.")

    def run_parametric_step(self):
        if not self.running:
            return

        self.test_index += 1

        # Perturbaciones aleatorias
        K_rand = BASE_K + random.uniform(-self.perturb_K, self.perturb_K)
        tau_rand = BASE_TAU + random.uniform(-self.perturb_tau, self.perturb_tau)

        # Crear sistema
        sys_rand = make_first_order_system(K_rand, tau_rand)

        # Entrada con ruido
        u = np.ones_like(TIME_VECTOR) * (TARGET_RPM / BASE_K)
        u += np.random.normal(0, self.noise_amp, size=len(u))

        t_out, y_out = simulate_forced_response(sys_rand, TIME_VECTOR, u)
        y_out = np.clip(y_out, 0, TARGET_RPM)

        # Calcular overshoot
        overshoot = max(0, ((max(y_out) - TARGET_RPM) / TARGET_RPM) * 100)

        # Actualizar gráficas
        line, = self.ax1.plot(t_out, y_out, lw=1.5)
        self.curve_lines.append(line)
        self.ax2.scatter(self.test_index, K_rand, color="#00C49A", label="K" if self.test_index == 1 else "")
        self.ax2.scatter(self.test_index, tau_rand, color="#FF6B6B", label="τ" if self.test_index == 1 else "")
        self.k_values.append(K_rand)
        self.tau_values.append(tau_rand)

        if self.test_index == 1:
            self.ax2.legend(facecolor="#0d1117", edgecolor="gray", labelcolor="white")

        self.canvas.draw()

        # Guardar datos
        self.data_records.append({
            "Test": self.test_index,
            "K": K_rand,
            "Tau": tau_rand,
            "Overshoot (%)": overshoot
        })

        self.table.insertRow(self.table.rowCount())
        self.table.setItem(self.table.rowCount()-1, 0, QtWidgets.QTableWidgetItem(str(self.test_index)))
        self.table.setItem(self.table.rowCount()-1, 1, QtWidgets.QTableWidgetItem(f"{K_rand:.3f}"))
        self.table.setItem(self.table.rowCount()-1, 2, QtWidgets.QTableWidgetItem(f"{tau_rand:.3f}"))
        self.table.setItem(self.table.rowCount()-1, 3, QtWidgets.QTableWidgetItem(f"{overshoot:.2f}"))

    def export_results(self):
        if not self.data_records:
            QtWidgets.QMessageBox.warning(self, "Sin datos", "No hay resultados para exportar.")
            return

        out_dir = os.path.join("outputs", "motor_malo_parametric_test")
        os.makedirs(out_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        df = pd.DataFrame(self.data_records)
        csv_path = os.path.join(out_dir, f"resultados_parametricos_{timestamp}.csv")
        png_path = os.path.join(out_dir, f"grafico_resumen_{timestamp}.png")

        df.to_csv(csv_path, index=False)
        self.fig.savefig(png_path, bbox_inches="tight")

        QtWidgets.QMessageBox.information(
            self, "Exportado",
            f"Resultados guardados en:\n{csv_path}\n{png_path}"
        )

def run_perturbation_test():
    K = BASE_K
    tau = BASE_TAU
    Ts = TIME_VECTOR[1] - TIME_VECTOR[0]
    ref = TARGET_RPM
    rpm = 0
    rpm_hist = []
    error_hist = []
    for i, t in enumerate(TIME_VECTOR):
        # Falla simulada entre 3 y 5 segundos
        falla = 3 < t < 5
        ruido_std = 5.0 if not falla else 20.0
        rpm = motor_malo_step(rpm, ref, K, tau, Ts, ruido_std=ruido_std, falla=falla)
        rpm_hist.append(rpm)
        error_hist.append(ref - rpm)
    mse = np.mean(np.square(error_hist))
    print(f"Error cuadrático medio bajo perturbaciones: {mse:.2f}")
    return rpm_hist, error_hist


# ======= MAIN =======
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    gui = MotorMaloParametricTestGUI()
    gui.show()
    sys.exit(app.exec_())
