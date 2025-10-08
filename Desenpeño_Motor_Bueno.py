import numpy as np
import matplotlib.pyplot as plt

# Parámetros del modelo
K = 1.0       # Ganancia
tau = 0.8     # Constante de tiempo
Ts = 0.01     # Paso de muestreo
t = np.arange(0, 10, Ts)
ref = 1000    # Referencia (RPM)

# Control PID
Kp, Ki, Kd = 0.6, 0.3, 0.05

# Inicialización
rpm_no_control = []
rpm_pid = []
u_pid = 0
rpm_nc = 0
rpm_c = 0
error_prev = 0
integral = 0

# Simulación
for _ in t:
    # Sin control
    rpm_nc = rpm_nc + (Ts/tau)*(-rpm_nc + K*ref)
    rpm_no_control.append(rpm_nc)

    # Con PID
    error = ref - rpm_c
    integral += error * Ts
    deriv = (error - error_prev) / Ts
    u_pid = Kp * error + Ki * integral + Kd * deriv
    rpm_c = rpm_c + (Ts/tau)*(-rpm_c + K*u_pid)
    rpm_pid.append(rpm_c)
    error_prev = error


def metrics(signal):
    steady_state = signal[-1]
    overshoot = (max(signal) - ref) / ref * 100
    settling_time = None
    for i in range(len(signal)):
        if abs(signal[i] - ref) < 0.02 * ref:
            if all(abs(x - ref) < 0.02 * ref for x in signal[i:]):
                settling_time = i * Ts
                break
    steady_error = ref - steady_state
    return settling_time, overshoot, steady_error


ts_nc, os_nc, err_nc = metrics(rpm_no_control)
ts_pid, os_pid, err_pid = metrics(rpm_pid)


def fmt(value):
    return f"{value:.2f}" if value is not None else "No estable"


fig1, ax1 = plt.subplots(figsize=(10, 6))
ax1.plot(t, rpm_no_control, 'r--', label='Sin control')
ax1.plot(t, rpm_pid, 'b-', linewidth=2, label='Con PID')
ax1.axhline(ref, color='gray', linestyle='--', label='Referencia')

ax1.set_title("Comparación del Motor Bueno: Sin Control vs PID")
ax1.set_xlabel("Tiempo [s]")
ax1.set_ylabel("Velocidad (RPM)")
ax1.legend()
ax1.grid(True)
plt.tight_layout()
plt.show()


fig2, ax2 = plt.subplots(figsize=(6, 2))
ax2.axis('off')  


column_labels = ['Métrica', 'Sin control', 'Con PID']
table_data = [
    ['Tiempo de establecimiento (s)', fmt(ts_nc), fmt(ts_pid)],
    ['Sobreimpulso (%)', fmt(os_nc), fmt(os_pid)],
    ['Error estacionario (RPM)', fmt(err_nc), fmt(err_pid)]
]


table = ax2.table(cellText=table_data, colLabels=column_labels, cellLoc='center',
                  loc='center', colWidths=[0.4, 0.3, 0.3])


table.auto_set_font_size(False)
table.set_fontsize(10)
table.scale(1.2, 1.6)


ax2.set_title("Métricas del Desempeño del Motor Bueno", fontsize=12, weight='bold', pad=10)

plt.tight_layout()
plt.show()
