def __init__(self, Kp, Ki, Kd, output_min=None, output_max=None, integral_limit=None):

    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd

    self.integral = 0.0
    self.last_error = 0.0
    
    self.output_min = output_min
    self.output_max = output_max
    
    self.integral_limit = integral_limit 

def calculate(self, setpoint, process_variable, dt):
    
    error = setpoint - process_variable

    P_term = self.Kp * error
    
    self.integral += error * dt
    
    if self.integral_limit is not None:
         self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

    I_term = self.Ki * self.integral

    D_term = 0.0
    if dt > 0:
        D_term = self.Kd * ((error - self.last_error) / dt)
    
    self.last_error = error

    control_output = P_term + I_term + D_term

    if self.output_min is not None and control_output < self.output_min:
        control_output = self.output_min
    if self.output_max is not None and control_output > self.output_max:
        control_output = self.output_max
        
    return control_output