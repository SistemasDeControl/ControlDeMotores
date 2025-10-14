# motor_malo_model.py
import random
import time
import numpy as np

YMAX_DATA = 15000.0

class MotorMalo:
    """Motor malo con perturbaciones realistas y dinámica suavizada"""
    def __init__(self, K=15000.0, tau=1.2, Ts=0.2):
        self.K = K
        self.tau = tau
        self.Ts = Ts
        self.rpm = 0.0
        self.last_u = 0.5  # Entrada inicial moderada
        self.start_time = time.time()

    def _apply_dynamics(self, u, perturbation_scale=1.0):
        """Modelo con dinámica de primer orden + perturbaciones suaves"""
        # Dinámica básica
        rpm_nominal = self.rpm + (self.Ts / self.tau) * (-self.rpm + self.K * u)

        # Perturbaciones suaves (ruido normal y variaciones de carga lentas)
        noise = np.random.normal(0, 50 * perturbation_scale)  # ruido pequeño
        load_variation = 150 * perturbation_scale * np.sin(time.time() * 0.3)  # carga sinusoidal lenta

        # Fricción variable ligera
        friction_factor = 1.0 - 0.002 * random.random() * perturbation_scale

        # Aplicar todo
        rpm_new = rpm_nominal * friction_factor + noise + load_variation

        # Saturación superior e inferior realista
        self.rpm = max(0.0, min(rpm_new, YMAX_DATA))
        return self.rpm

    def step_pid(self, u_pid):
        """Actualiza el motor con PID activo y perturbaciones moderadas"""
        u_pid = max(0.0, min(u_pid, 1.0))  # acotar entre 0 y 1
        return self._apply_dynamics(u_pid, perturbation_scale=0.5)

    def step_aleatorio(self):
        """Modo aleatorio: entrada varía lentamente, perturbaciones realistas"""
        # Cambio gradual de la entrada
        target_u = random.uniform(0.3, 0.9)  # evita valores demasiado bajos
        self.last_u += 0.02 * (target_u - self.last_u)  # suaviza cambio
        rpm = self._apply_dynamics(self.last_u, perturbation_scale=0.5)
        return rpm, self.last_u

    def reset(self):
        self.rpm = 0.0
        self.last_u = 0.5
        self.start_time = time.time()
