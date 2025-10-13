import numpy as np

class PIDController:
    """
    Controlador PID simple.
    """
    def __init__(self, Kp, Ki, Kd, output_min=None, output_max=None, integral_limit=None):
        """
        Inicializa el controlador PID.

        Args:
            Kp (float): Ganancia proporcional.
            Ki (float): Ganancia integral.
            Kd (float): Ganancia derivativa.
            output_min (float, optional): Límite inferior para la salida del controlador.
            output_max (float, optional): Límite superior para la salida del controlador.
            integral_limit (float, optional): Límite para la acumulación integral (anti-windup).
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral = 0.0
        self.last_error = 0.0
        
        self.output_min = output_min
        self.output_max = output_max
        
        self.integral_limit = integral_limit 

    def calculate(self, setpoint, process_variable, dt):
        """
        Calcula la salida del controlador PID.

        Args:
            setpoint (float): El valor deseado.
            process_variable (float): El valor actual medido.
            dt (float): El intervalo de tiempo desde la última llamada (en segundos).

        Returns:
            float: La salida de control calculada.
        """
        error = setpoint - process_variable

        # Término Proporcional
        P_term = self.Kp * error
        
        # Término Integral
        self.integral += error * dt
        
        # Anti-windup para el término integral
        if self.integral_limit is not None:
             self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        I_term = self.Ki * self.integral

        # Término Derivativo
        D_term = 0.0
        if dt > 0: # Evitar división por cero
            D_term = self.Kd * ((error - self.last_error) / dt)
        
        self.last_error = error

        # Suma de los términos
        control_output = P_term + I_term + D_term

        # Saturación de la salida (si se definieron límites)
        if self.output_min is not None and control_output < self.output_min:
            control_output = self.output_min
        if self.output_max is not None and control_output > self.output_max:
            control_output = self.output_max
            
        return control_output

    def reset(self):
        
        self.integral = 0.0
        self.last_error = 0.0