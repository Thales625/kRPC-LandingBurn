class PIDController:
    def __init__(self):
        self.min_out_limit, self.max_out_limit = 0.0, 1.0
        self.kp, self.ki, self.kd = 0.025, 0.001, 0.1
        self.proportional_term, self.integral_term, self.derivative_term = 0.0, 0.0, 0.0
        self.last_error, self.last_time = 0.0, None
        self.time_sample = 0.025

    def __call__(self, current_value, set_point, now):
        if self.last_time is not None:
            delta_time = float(now - self.last_time)
        else:
            delta_time = 0.1

        if delta_time >= self.time_sample:
            error = set_point - current_value

            delta_error = error - self.last_error

            self.proportional_term = self.kp * error
            self.integral_term = self.limit_value(self.integral_term + self.ki * delta_time * error)
            self.derivative_term = self.kd * delta_error / delta_time

            self.last_time = now
            self.last_error = error

        return self.limit_value(self.proportional_term + self.integral_term + self.derivative_term)
    
    def limit_value(self, value):
        return max(self.min_out_limit, min(self.max_out_limit, value))

    def limit_output(self, _min, _max):
        if _min > _max:
            return
        self.min_out_limit = _min
        self.max_out_limit = _max
        self.integral_term = self.limit_value(self.integral_term)

    def adjust_gains(self, kp=None, ki=None, kd=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

class PIController:
    def __init__(self):
        self.min_out_limit, self.max_out_limit = 0.0, 1.0
        self.kp, self.ki = 0.025, 0.001
        self.proportional_term, self.integral_term = 0.0, 0.0
        self.last_time = None
        self.time_sample = 0.025

    def __call__(self, current_value, set_point, now):
        if self.last_time is not None:
            delta_time = float(now - self.last_time)
        else:
            delta_time = 0.1

        if delta_time >= self.time_sample:
            error = set_point - current_value

            self.proportional_term = self.kp * error
            self.integral_term = self.limit_value(self.integral_term + self.ki * delta_time * error)

            self.last_time = now

        return self.limit_value(self.proportional_term + self.integral_term)
    
    def limit_value(self, value):
        return max(self.min_out_limit, min(self.max_out_limit, value))

    def limit_output(self, _min, _max):
        if _min > _max:
            return
        self.min_out_limit = _min
        self.max_out_limit = _max
        self.integral_term = self.limit_value(self.integral_term)

    def adjust_gains(self, kp=None, ki=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki

class PDController:
    def __init__(self):
        self.min_out_limit, self.max_out_limit = 0.0, 1.0
        self.kp, self.kd = 0.025, 0.1
        self.proportional_term, self.derivative_term = 0.0, 0.0
        self.last_error, self.last_time = 0.0, None
        self.time_sample = 0.025

    def __call__(self, current_value, set_point, now):
        if self.last_time is not None:
            delta_time = float(now - self.last_time)
        else:
            delta_time = 0.1

        if delta_time >= self.time_sample:
            error = set_point - current_value

            delta_error = error - self.last_error

            self.proportional_term = self.kp * error
            self.derivative_term = self.kd * delta_error / delta_time

            self.last_time = now
            self.last_error = error

        return self.limit_value(self.proportional_term + self.derivative_term)
    
    def limit_value(self, value):
        return max(self.min_out_limit, min(self.max_out_limit, value))

    def limit_output(self, _min, _max):
        if _min > _max:
            return
        self.min_out_limit = _min
        self.max_out_limit = _max

    def adjust_gains(self, kp=None, kd=None):
        if kp is not None:
            self.kp = kp
        if kd is not None:
            self.kd = kd