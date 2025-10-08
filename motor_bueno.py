import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import tkinter as tk
from threading import Thread
import time


# Modelo matemático del motor (primer orden discretizado)
def motor_step(rpm_prev, u, K, tau, Ts):
    rpm = rpm_prev + (Ts/tau) * (-rpm_prev + K * u)
    return rpm


# Clase de simulación con control de encendido/apagado

class MotorSimulator:
    def __init__(self, K=100, tau=0.5, Ts=0.05):
        self.K = K
        self.tau = tau
        self.Ts = Ts
        self.rpm = 0
        self.running = False  # motor apagado inicialmente
        self.time = [0]
        self.rpm_history = [0]
        self.u_history = [0]
        self.start_time = time.time()

    def step(self):
        u = 1.0 if self.running else 0.0
        self.rpm = motor_step(self.rpm, u, self.K, self.tau, self.Ts)
        t = time.time() - self.start_time
        self.time.append(t)
        self.rpm_history.append(self.rpm)
        self.u_history.append(u)


# Animación de Matplotlib

def animate(i, sim, ax1, ax2):
    sim.step()
    ax1.clear()
    ax2.clear()

    ax1.plot(sim.time, sim.rpm_history, label="Motor (RPM)", color="blue")
    ax1.set_ylabel("RPM")
    ax1.set_title("Motor con Control ON/OFF")
    ax1.grid(True)
    ax1.legend()

    ax2.plot(sim.time, sim.u_history, label="Entrada (u)", color="green")
    ax2.set_xlabel("Tiempo [s]")
    ax2.set_ylabel("Potencia Normalizada")
    ax2.grid(True)
    ax2.legend()


# Interfaz Tkinter (Botones ON/OFF)

def start_gui(sim):
    def turn_on():
        sim.running = True
        print("Motor encendido")

    def turn_off():
        sim.running = False
        print("Motor apagado")

    root = tk.Tk()
    root.title("Control del Motor")

    btn_on = tk.Button(root, text="Encender Motor", command=turn_on, bg="lightgreen", width=20)
    btn_on.pack(pady=10)

    btn_off = tk.Button(root, text="Apagar Motor", command=turn_off, bg="lightcoral", width=20)
    btn_off.pack(pady=10)

    root.mainloop()


# Main

if __name__ == "__main__":
    sim = MotorSimulator()

    # Ventana de matplotlib
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8,6))

    ani = animation.FuncAnimation(
        fig, animate, fargs=(sim, ax1, ax2), interval=50,
        cache_frame_data=False  # evita warning de cache
    )

    # Tkinter en hilo paralelo (para que puedas interactuar)
    gui_thread = Thread(target=start_gui, args=(sim,), daemon=True)
    gui_thread.start()

    plt.tight_layout()

    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n⚠️ Simulación detenida manualmente.")
