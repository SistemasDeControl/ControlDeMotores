# bad_motor_adapter.py
from dataclasses import dataclass
import math
import random

def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)

@dataclass
class BadMotorConfig:
    open_loop_rpm: float = 10000.0      # objetivo en ABIERTO
    k_motor: float = 15000.0            # igual a MotorMalo.K
    Ts: float = 0.2

    noise_amp_u: float = 0.05           # ruido coloreado (media 0)
    noise_bandwidth_hz: float = 0.35
    step_prob: float = 0.06             # escalones ±step_u con decaimiento
    step_u: float = 0.12
    step_tau_s: float = 2.0

class BadMotorAdapter:
    """
    API:
      - reset()
      - step_aleatorio() -> (rpm, u_aplicada)
      - step_pid(u_pid)  -> rpm
      - rpm (propiedad)
      - last_u_eff
    """
    def __init__(self, motor_malo, cfg: BadMotorConfig):
        self.m = motor_malo
        self.cfg = cfg

        self._u_noise = 0.0
        self._u_step  = 0.0
        self.last_u_eff = 0.0

        # AR(1) para ruido coloreado
        bw = max(1e-3, cfg.noise_bandwidth_hz)
        self._alpha = math.exp(-2.0 * math.pi * bw * cfg.Ts)
        self._beta  = math.sqrt(max(1e-8, 1.0 - self._alpha**2))

        # Bases y topes
        self._u_base = _clamp(cfg.open_loop_rpm / max(1e-6, cfg.k_motor), 0.0, 1.0)
        self._u_max_open = self._u_base  # techo duro en ABIERTO para no superar ~10000 RPM

    @property
    def rpm(self) -> float:
        return getattr(self.m, "rpm", 0.0)

    def reset(self):
        self.m.reset()
        self._u_noise = 0.0
        self._u_step  = 0.0
        self.last_u_eff = 0.0

    def step_aleatorio(self):
        """
        ABIERTO: u = clamp( u_base + perturbación, 0, u_max_open )
        Mantiene ruido y escalones pero NO permite superar ~10000 RPM.
        """
        u_dist = self._next_disturbance_u()
        u = _clamp(self._u_base + u_dist, 0.0, self._u_max_open)
        rpm = self.m.step_pid(u)     # integrador interno con 'u'
        self.last_u_eff = u
        return rpm, u

    def step_pid(self, u_pid: float):
        """
        CON PID: u_eff = clamp( u_pid + perturbación, 0, 1 ).
        El integrador del PID elimina error estacionario y llega a 15000 RPM.
        """
        u_eff = _clamp(u_pid + self._next_disturbance_u(), 0.0, 1.0)
        rpm = self.m.step_pid(u_eff)
        self.last_u_eff = u_eff
        return rpm

    def _next_disturbance_u(self) -> float:
        # Ruido coloreado de media 0
        self._u_noise = self._alpha * self._u_noise + self._beta * self.cfg.noise_amp_u * random.gauss(0.0, 1.0)

        # Escalón aleatorio ±step_u
        if random.random() < self.cfg.step_prob:
            self._u_step += (1.0 if random.random() < 0.5 else -1.0) * self.cfg.step_u

        # Decaimiento exponencial
        self._u_step *= math.exp(-self.cfg.Ts / max(1e-3, self.cfg.step_tau_s))

        # Perturbación total (media ~0)
        return self._u_noise + self._u_step
#MOTRO MALO PID 15000 RPM