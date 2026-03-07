from config import * # lsp-only
from pid import PIDController # lsp-only

class DriveState:
    """Holds per-command state for the non-blocking driveToPoint."""
    def __init__(self):
        self.initialized = False
        self.settle_counter = 0
        self.start_time = 0

    def reset(self):
        chassis_x_pid.reset()
        chassis_y_pid.reset()
        chassis_heading_pid.reset()
        self.settle_counter = 0
        self.start_time = brain.timer.time()
        self.initialized = True

drive_state = DriveState()
team_color = "red"
chassis_x_pid = PIDController(kp=1.0, ki=0.05, kd=0.15, max_output=100)
chassis_y_pid = PIDController(kp=1.0, ki=0.05, kd=0.15, max_output=100)
chassis_heading_pid = PIDController(kp=1.0, ki=0.0, kd=0.2, max_output=50)
imu.calibrate()
while imu.is_calibrating():
    brain.screen.print("Calibrating IMU...")
    wait(100, MSEC)

gps.calibrate()
while gps.is_calibrating():
    brain.screen.print("Calibrating GPS...")
    wait(100, MSEC)

