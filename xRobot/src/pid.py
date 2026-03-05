from utils import * # lsp-only    
MAX_DT = 0.05  # seconds – cap dt so a stale timestamp can't blow up I/D

class PIDController:
    """PID controller for chassis motion"""
    def __init__(self, kp, ki, kd, max_output=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0
        self.previous_error = 0
        self.last_time = 0
        self._fresh = True  # True until the first real tick after reset

    def _step(self, error, timestamp, wrap_angle=False):
        fresh = self._fresh
        if self.last_time == 0:
            dt = 0.02
        else:
            dt = (timestamp - self.last_time) / 1000.0
        # Cap dt so a large gap doesn't blow up integral / derivative
        if dt > MAX_DT:
            dt = MAX_DT

        p_output = self.kp * error

        self.integral += error * dt
        # Clamp the integral contribution, not the raw accumulator
        i_output = self.ki * self.integral
        i_output = max(min(i_output, self.max_output), -self.max_output)
        if i_output != 0:
            self.integral = i_output / self.ki

        # Skip derivative on the first tick – no valid previous_error yet
        if fresh or dt <= 0:
            derivative = 0
        else:
            delta_error = error - self.previous_error
            if wrap_angle:
                while delta_error > 180:
                    delta_error -= 360
                while delta_error <= -180:
                    delta_error += 360
            derivative = delta_error / dt
        d_output = self.kd * derivative

        output = p_output + i_output + d_output
        output = max(min(output, self.max_output), -self.max_output)

        self.previous_error = error
        self.last_time = timestamp
        self._fresh = False
        return output

    def calculate(self, target_value, current_value, timestamp):
        error = target_value - current_value
        return self._step(error, timestamp)

    def calculate_error(self, error, timestamp):
        return self._step(error, timestamp)

    def calculate_angle_error(self, error, timestamp):
        """PID step for angular values – normalises the derivative delta
        so that crossing the ±180° boundary doesn't cause a spike."""
        return self._step(error, timestamp, wrap_angle=True)

    def reset(self):
        self.integral = 0
        self.previous_error = 0
        self.last_time = 0
        self._fresh = True