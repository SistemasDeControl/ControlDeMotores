import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import matplotlib.patches as patches
import random
import time
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PIL import Image, ImageSequence
from pid_controller import PIDController 

# ==========================================================
# ==========   GIF DEL MOTOR (COMPARTIDO)   ================
# (Código del GIF sin cambios)
# ==========================================================
# ... (Código del GIF) ...
gif_frames = []
try:
    gif = Image.open("motor.gif")
    for frame in ImageSequence.Iterator(gif):
        frame = frame.convert("RGBA")
        gif_frames.append(np.array(frame))
    gif_idx_malo = 0
    gif_idx_bueno = 0
except Exception as e:
    print("⚠️ No se pudo cargar motor.gif:", e)
    gif_frames = []
    gif_idx_malo = 0
    gif_idx_bueno = 0

# ==========================================================
# =======   CONFIGURACIÓN DEL CONTROL Y MOTOR MALO   =======
# ==========================================================
# Parámetros del Motor Malo (Lento y con poca ganancia, ej. K=600, tau=1.5)
K_malo, tau_malo, Ts_malo = 600.0, 1.5, 0.2 
motor_rpm_malo = 0.0 # RPM actual del motor malo
u_malo = 0.0         # Voltaje (entrada) actual al motor malo
referencia_malo = 800.0 # RPM de referencia (Setpoint)

motor_on_malo = False
pid_activo_malo = False # NUEVA BANDERA para activar el PID
tiempo_malo = 0.0
hist_time_malo, hist_rpm_malo, hist_u_malo = [], [], []

# Inicializar el Controlador PID
# **AJUSTA ESTAS GANANCIAS** para el Motor Malo (usa las que sintonizaste)
Kp_M, Ki_M, Kd_M = 0.5, 0.1, 0.05 
PID_M = PIDController(Kp_M, Ki_M, Kd_M, dt=Ts_malo, limite_u=1.0) # El límite es 1.0 (PWM) * 1200 RPM

def motor_malo_step(rpm_prev, u, K, tau, Ts):
    """Modelo discreto simple para el Motor Malo."""
    rpm_new = rpm_prev + (Ts/tau) * (-rpm_prev + K * u)
    return max(0.0, min(rpm_new, 1200.0))


def motor_step(rpm_prev, u, K, tau, Ts):
    rpm_new = rpm_prev + (Ts/tau) * (-rpm_prev + K * u)
    return max(0.0, min(rpm_new, 1200.0))

class MotorBueno:
    def __init__(self, K=1200.0, tau=0.8, Ts=0.1):
        self.K = K
        self.tau = tau
        self.Ts = Ts
        self.rpm = 0.0
        self.running = False
        self.start_time = time.time()
        self.time = []
        self.rpm_history = []
        self.u_history = []

    def step(self):
        u = 1.0 if self.running else 0.0
        self.rpm = motor_step(self.rpm, u, self.K, self.tau, self.Ts)
        t = time.time() - self.start_time
        self.time.append(t)
        self.rpm_history.append(min(self.rpm, 1200.0))
        self.u_history.append(u)

sim_bueno = MotorBueno()

# ==========================================================
# ===============   FIGURA GENERAL (SIMÉTRICA)  ============
# ... (Código de Subplots y configuración general) ...
# ==========================================================
fig = plt.figure(figsize=(14, 10))

ax_img_malo   = plt.subplot2grid((3,2),(0,0))
ax_img_bueno  = plt.subplot2grid((3,2),(0,1))
ax_table_malo = plt.subplot2grid((3,2),(1,0))
ax_table_bueno= plt.subplot2grid((3,2),(1,1))
ax_graph_malo = plt.subplot2grid((3,2),(2,0))
ax_graph_bueno= plt.subplot2grid((3,2),(2,1))

plt.subplots_adjust(bottom=0.18, hspace=0.6, wspace=0.4)

# ... (Configuración de Dibujo y Tablas sin cambios) ...
# ==========================================================
# ===============   DIBUJO MOTOR MALO   ====================
# ==========================================================
motor_circle_malo = patches.Circle((0.5, 0.5), 0.2, color='gray', ec='black')
ax_img_malo.add_patch(motor_circle_malo)
status_text_malo = ax_img_malo.text(0.5, 0.85, "Motor Malo: APAGADO",
                                     ha="center", fontsize=11, color="red")

motor_imagebox_malo = None
if gif_frames:
    imagebox = OffsetImage(gif_frames[0], zoom=0.25)
    motor_imagebox_malo = AnnotationBbox(imagebox, (0.5, 0.5), frameon=False)
    ax_img_malo.add_artist(motor_imagebox_malo)

ax_img_malo.set_xlim(0,1); ax_img_malo.set_ylim(0,1)
ax_img_malo.axis("off")

# Tabla motor malo
column_labels = ["Métrica", "Valor"]
metrics_malo = [
    ["Tiempo (s)", "0.00"],
    ["Entrada (u)", "0.00"],
    ["RPM actuales", "0.00"],
    ["Sobreimpulso (%)", "0.00"],
    ["Tiempo subida (s)", "0.00"],
    ["Tiempo asentamiento (s)", "0.00"],
    ["Error estacionario", "0.00"],
]
table_malo = ax_table_malo.table(cellText=metrics_malo, colLabels=column_labels, loc="center", cellLoc="center")
table_malo.scale(1.3, 1.5)
ax_table_malo.axis("off")

# ... (Configuración de Motor Bueno sin cambios) ...
# ==========================================================
# ===============   DIBUJO MOTOR BUENO   ===================
# ==========================================================
motor_circle_bueno = patches.Circle((0.5, 0.5), 0.2, color='gray', ec='black')
ax_img_bueno.add_patch(motor_circle_bueno)
status_text_bueno = ax_img_bueno.text(0.5, 0.85, "Motor Bueno: APAGADO",
                                     ha="center", fontsize=11, color="red")

motor_imagebox_bueno = None
if gif_frames:
    imagebox = OffsetImage(gif_frames[0], zoom=0.25)
    motor_imagebox_bueno = AnnotationBbox(imagebox, (0.5, 0.5), frameon=False)
    ax_img_bueno.add_artist(motor_imagebox_bueno)

ax_img_bueno.set_xlim(0,1); ax_img_bueno.set_ylim(0,1)
ax_img_bueno.axis("off")

# Tabla motor bueno
metrics_bueno = [
    ["Tiempo (s)", "0.00"],
    ["Entrada (u)", "0.00"],
    ["RPM actuales", "0.00"],
    ["Sobreimpulso (%)", "0.00"],
    ["Tiempo subida (s)", "0.00"],
    ["Tiempo asentamiento (s)", "0.00"],
    ["Error estacionario", "0.00"],
]
table_bueno = ax_table_bueno.table(cellText=metrics_bueno, colLabels=column_labels, loc="center", cellLoc="center")
table_bueno.scale(1.3, 1.5)
ax_table_bueno.axis("off")

# ... (Configuración de Gráficas sin cambios) ...
# ==========================================================
# ===============   GRAFICAS (MISMO FORMATO)  ==============
# ==========================================================
YMAX = 1200.0
XWINDOW = 40

line_rpm_malo, = ax_graph_malo.plot([], [], label="RPM", color="red", linewidth=2)
line_u_malo,   = ax_graph_malo.plot([], [], label="Entrada (u)*1200", linestyle="--", color="black")
ax_graph_malo.set_ylim(0, YMAX)
ax_graph_malo.set_xlim(0, XWINDOW)
ax_graph_malo.set_title("Motor Malo", fontsize=12)
ax_graph_malo.set_xlabel("Tiempo [s]")
ax_graph_malo.set_ylabel("RPM")
ax_graph_malo.legend(); ax_graph_malo.grid(True)

line_rpm_bueno, = ax_graph_bueno.plot([], [], label="RPM", color="blue", linewidth=2)
line_u_bueno,   = ax_graph_bueno.plot([], [], label="Entrada (u)*1200", linestyle="--", color="black")
ax_graph_bueno.set_ylim(0, YMAX)
ax_graph_bueno.set_xlim(0, XWINDOW)
ax_graph_bueno.set_title("Motor Bueno", fontsize=12)
ax_graph_bueno.set_xlabel("Tiempo [s]")
ax_graph_bueno.set_ylabel("RPM")
ax_graph_bueno.legend(); ax_graph_bueno.grid(True)


# ==========================================================
# ===============   FUNCIONES UPDATE (MODIFICADO)   =========
# ==========================================================
start_time = time.time()
def update(frame):
    global tiempo_malo, gif_idx_malo, gif_idx_bueno, motor_rpm_malo, u_malo

    t_now = time.time() - start_time
    Ts = 0.2 # Intervalo de la animación (usado como paso de tiempo de simulación)

    # -------- MOTOR MALO --------
    if motor_on_malo:
        # ----------------------------------------------------
        # 🔥🔥🔥 CÓDIGO CLAVE DEL PID 🔥🔥🔥
        # ----------------------------------------------------
        if pid_activo_malo:
            # 1. El PID calcula el nuevo voltaje (u_malo)
            u_malo = PID_M.calcular_salida(referencia_malo, motor_rpm_malo)
            # 2. El motor avanza un paso con ese voltaje (u_malo)
            motor_rpm_malo = motor_malo_step(motor_rpm_malo, u_malo, K_malo, tau_malo, Ts_malo)
            
            # (El texto del botón puede cambiar en la función del botón)

        else: 
            # Modo Lazo Abierto (o con ruido, como estaba antes)
            # Solo si el motor debe estar encendido (u=1.0)
            u_malo = 1.0 
            motor_rpm_malo = motor_malo_step(motor_rpm_malo, u_malo, K_malo, tau_malo, Ts_malo)
            
            # Opcional: añadir ruido para simular el comportamiento 'malo'
            motor_rpm_malo += random.uniform(-100, 100)
            motor_rpm_malo = max(0, min(motor_rpm_malo, YMAX))
            # u_malo = 1.0 if random.random() < 0.7 else 0.0 # Opción ruído de voltaje
        
        rpm_m = motor_rpm_malo
        
        tiempo_malo += Ts
        hist_time_malo.append(t_now)
        hist_rpm_malo.append(min(rpm_m, YMAX))
        hist_u_malo.append(u_malo)

        while len(hist_time_malo) > 0 and (hist_time_malo[-1] - hist_time_malo[0]) > XWINDOW:
            hist_time_malo.pop(0); hist_rpm_malo.pop(0); hist_u_malo.pop(0)

        times_rel = [tt - hist_time_malo[0] for tt in hist_time_malo]
        line_rpm_malo.set_data(times_rel, hist_rpm_malo)
        line_u_malo.set_data(times_rel, [uu * YMAX for uu in hist_u_malo])

        motor_circle_malo.set_color("green")
        status_text_malo.set_text(f"Motor Malo: ENCENDIDO (PID: {'ON' if pid_activo_malo else 'OFF'})")
        status_text_malo.set_color("green")

        if gif_frames and motor_imagebox_malo:
            gif_idx_malo = (gif_idx_malo + 1) % len(gif_frames)
            motor_imagebox_malo.offsetbox = OffsetImage(gif_frames[gif_idx_malo], zoom=0.25)

        # actualizar tabla
        # Se requiere lógica para calcular métricas de respuesta aquí (no solo aleatorias)
        last_t = times_rel[-1] if times_rel else 0
        last_u, last_rpm = hist_u_malo[-1], hist_rpm_malo[-1]
        
        # Estas métricas siguen siendo aleatorias, deberían calcularse de hist_rpm_malo
        vals = [last_t, last_u, last_rpm, random.uniform(5,20), random.uniform(0.5,2), random.uniform(2,6), (referencia_malo - last_rpm)/referencia_malo]
        for i, v in enumerate(vals):
            table_malo._cells[(i+1,1)].get_text().set_text(f"{v:.2f}")
    else:
        # Si el motor está apagado, se resetea todo
        u_malo = 0.0
        motor_rpm_malo = 0.0
        PID_M.integral = 0.0 # Reset de la integral es clave al apagar
        PID_M.error_anterior = 0.0
        motor_circle_malo.set_color("gray")
        status_text_malo.set_text(f"Motor Malo: APAGADO (PID: {'ON' if pid_activo_malo else 'OFF'})")
        status_text_malo.set_color("red")
        if gif_frames and motor_imagebox_malo:
            motor_imagebox_malo.offsetbox = OffsetImage(gif_frames[0], zoom=0.25)


    # -------- MOTOR BUENO --------
    # ... (Código de Motor Bueno sin cambios) ...
    sim_bueno.step()
    if sim_bueno.rpm_history: sim_bueno.rpm_history[-1] = min(sim_bueno.rpm_history[-1], YMAX)

    if sim_bueno.running:
        motor_circle_bueno.set_color("green")
        status_text_bueno.set_text("Motor Bueno: ENCENDIDO")
        status_text_bueno.set_color("green")
    else:
        motor_circle_bueno.set_color("gray")
        status_text_bueno.set_text("Motor Bueno: APAGADO")
        status_text_bueno.set_color("red")

    if sim_bueno.time:
        times = np.array(sim_bueno.time)
        t0 = times[0]
        times_rel = times - t0
        mask = (times_rel >= max(0, times_rel[-1] - XWINDOW))
        times_rel_win = times_rel[mask]
        rpm_win = np.array(sim_bueno.rpm_history)[mask]
        u_win   = np.array(sim_bueno.u_history)[mask]

        line_rpm_bueno.set_data(times_rel_win, rpm_win)
        line_u_bueno.set_data(times_rel_win, u_win * YMAX)

        # actualizar tabla
        last_t = times_rel_win[-1] if len(times_rel_win) else 0.0
        last_u = float(u_win[-1]) if len(u_win) else 0.0
        last_rpm = float(rpm_win[-1]) if len(rpm_win) else 0.0
        vals = [last_t, last_u, last_rpm, random.uniform(5,15), random.uniform(0.5,1.5), random.uniform(2,4), random.uniform(0,0.1)]
        for i, v in enumerate(vals):
            table_bueno._cells[(i+1,1)].get_text().set_text(f"{v:.2f}")

        if gif_frames and motor_imagebox_bueno and sim_bueno.running:
            gif_idx_bueno = (gif_idx_bueno + 1) % len(gif_frames)
            motor_imagebox_bueno.offsetbox = OffsetImage(gif_frames[gif_idx_bueno], zoom=0.25)
        elif gif_frames and motor_imagebox_bueno:
            motor_imagebox_bueno.offsetbox = OffsetImage(gif_frames[0], zoom=0.25)

    return (line_rpm_malo, line_u_malo, line_rpm_bueno, line_u_bueno)

ax_enc_malo = plt.axes([0.12, 0.03, 0.15, 0.06])
ax_apag_malo= plt.axes([0.30, 0.03, 0.15, 0.06])
ax_pid_malo = plt.axes([0.12, 0.10, 0.33, 0.06]) # Nuevo botón para el PID

btn_enc_malo= Button(ax_enc_malo, "Encender Malo", color="lightgreen")
btn_apag_malo= Button(ax_apag_malo, "Apagar Malo", color="lightcoral")
btn_pid_malo= Button(ax_pid_malo, "Aplicar PID (OFF)", color="lightblue") # Inicialmente OFF

ax_enc_bueno= plt.axes([0.55, 0.03, 0.15, 0.06])
ax_apag_bueno=plt.axes([0.73, 0.03, 0.15, 0.06])
btn_enc_bueno= Button(ax_enc_bueno, "Encender Bueno", color="lightgreen")
btn_apag_bueno= Button(ax_apag_bueno, "Apagar Bueno", color="lightcoral")

def encender_malo(event): global motor_on_malo; motor_on_malo=True
def apagar_malo(event): global motor_on_malo; motor_on_malo=False

def toggle_pid_malo(event):
    """Activa/Desactiva el control PID y resetea la memoria integral."""
    global pid_activo_malo
    pid_activo_malo = not pid_activo_malo
    
    # Resetear el PID al cambiar de modo para evitar un salto brusco.
    PID_M.integral = 0.0
    PID_M.error_anterior = 0.0

    if pid_activo_malo:
        btn_pid_malo.label.set_text("Aplicar PID (ON) - Setpoint: 600 RPM")
        btn_pid_malo.color = "gold" # Cambia el color para indicar que está activo
    else:
        btn_pid_malo.label.set_text("Aplicar PID (OFF)")
        btn_pid_malo.color = "lightblue"


def encender_bueno(event): sim_bueno.running=True
def apagar_bueno(event):  sim_bueno.running=False

btn_enc_malo.on_clicked(encender_malo); btn_apag_malo.on_clicked(apagar_malo)
btn_enc_bueno.on_clicked(encender_bueno); btn_apag_bueno.on_clicked(apagar_bueno)
btn_pid_malo.on_clicked(toggle_pid_malo) # Conectar el nuevo botón

# ==========================================================
# ===============   ANIMACIÓN GENERAL   ====================
# ==========================================================
ani = FuncAnimation(fig, update, interval=Ts_malo*1000, blit=False) 
plt.show()