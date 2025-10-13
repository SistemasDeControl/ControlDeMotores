class PIDController:
    def __init__(self, Kp, Ki, Kd, dt, limite_u=12.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.limite_u = limite_u  

        self.integra
        l = 0.0
        self.error_anterior = 0.0

    def calcular_salida(self, referencia, salida_actual):
        # 1. Calcular el Error
        error = referencia - salida_actual

        # 2. Acción Proporcional (P)
        ter_p = self.Kp * error

        # 3. Acción Integral (I): Acumulación y Anti-Windup
        self.integral += error * self.dt  
        
        # Anti-Windup Simple: Limita la acumulación de la integral 
        # si esta va a exceder el límite de la salida de control.
        if self.Ki != 0:
            if self.integral > self.limite_u / self.Ki:
                self.integral = self.limite_u / self.Ki
            elif self.integral < -self.limite_u / self.Ki:
                self.integral = -self.limite_u / self.Ki
                
        ter_i = self.Ki * self.integral

        # 4. Acción Derivativa (D)
        if self.dt > 0:
            derivada = (error - self.error_anterior) / self.dt
        else:
            derivada = 0.0
        ter_d = self.Kd * derivada

        # 5. Salida de Control (Suma P + I + D)
        salida_sin_limite = ter_p + ter_i + ter_d
        
        # 6. Saturación de Salida: Asegura que el control no exceda los límites físicos.
        salida_control = max(min(salida_sin_limite, self.limite_u), -self.limite_u)

        # 7. Actualizar el estado para el próximo ciclo
        self.error_anterior = error

        return salida_control