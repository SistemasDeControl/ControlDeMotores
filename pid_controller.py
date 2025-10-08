class PIDController:
    """
    Controlador PID: El 'cerebro' que calcula el voltaje para que el motor
    alcance la velocidad deseada (Referencia).
    """
    def __init__(self, Kp, Ki, Kd, dt, limite_u=12.0):
        # 1. Ajustes del Controlador (Ganancias)
        self.Kp = Kp        # Ganancia P (Impulso principal)
        self.Ki = Ki        # Ganancia I (Elimina errores pequeños)
        self.Kd = Kd        # Ganancia D (Evita que se pase de largo)
        self.dt = dt        # Cuánto dura cada paso de tiempo (simulación)
        self.limite_u = limite_u # Voltaje máximo que podemos enviar al motor (ej. 12V)
        self.integral = 0.0     # Acumulador del error (para la acción I)
        self.error_anterior = 0.0 # Error del ciclo pasado (para la acción D)

    def calcular_salida(self, referencia, salida_actual):
        """Calcula el voltaje de salida basándose en la diferencia entre 
        lo que queremos (referencia) y lo que tenemos (salida_actual)."""
        
        # 1. Calcular el Error
        error = referencia - salida_actual # Error = Lo que falta por alcanzar
        # 2. Acción Proporcional (P)
        ter_p = self.Kp * error # Reacción instantánea al error.
        # 3. Acción Integral (I)
        self.integral += error * self.dt # Acumula el error en el tiempo.

        # Anti-Windup Simple: Evita que la memoria (integral) crezca demasiado 
        # si el voltaje ya está al máximo.
        if self.Ki != 0:
            limite_integral = self.limite_u / self.Ki
            self.integral = max(min(self.integral, limite_integral), -limite_integral)
            # Nota: Usamos max(min()) para limitar la integral en un rango.
                
        ter_i = self.Ki * self.integral # Contribución para eliminar el error residual.

        # 4. Acción Derivativa (D)
        if self.dt > 0:
            # Calcula qué tan rápido está cambiando el error.
            derivada = (error - self.error_anterior) / self.dt
        else:
            derivada = 0.0
        ter_d = self.Kd * derivada # Contribución para frenar el sistema.
        # 5. Salida de Control (Suma P + I + D)
        salida_sin_limite = ter_p + ter_i + ter_d 
        # 6. Saturación de Salida: Corta el voltaje si excede el límite (ej. 12V).
        salida_control = max(min(salida_sin_limite, self.limite_u), -self.limite_u)
        self.error_anterior = error # Guarda el error actual para el cálculo D futuro.

        return salida_control