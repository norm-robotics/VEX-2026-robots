# ---------------------------------------------------------------------------- #
#   AUTO-GENERATED FILE – do not edit directly.                                #
#   Edit files in src/modules/ and run:                                        #
#       python build.py <robot_dir>                                            #
# ---------------------------------------------------------------------------- #

from vex import *
import math


# ===== config =====

# Library imports


# Brain and controller should be defined by default
brain=Brain()
controller = Controller()

#================================
#DEFINING MOTORS
#================================
#Bottom DriveTrain Motors
lBottFront = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
lBottBack = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
rBottFront = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
rBottBack = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)

#Upper DriveTrain Motors
lUpFront = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
lUpBack = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
rUpFront = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
rUpBack = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)

#Intake Motors
intake = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)

#Outtake Motors
outFlex = Motor(Ports.PORT21, GearSetting.RATIO_6_1, False)
outFlap = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)

#sensor Motors
gps = Gps(Ports.PORT20)
imu = Inertial(Ports.PORT3)

#Pneumatics
matchLoad = Pneumatics(brain.three_wire_port.a)
heightMech = Pneumatics(brain.three_wire_port.b)
descore = Pneumatics(brain.three_wire_port.c)

#MotorGroups (MG)
rearLeftMG = MotorGroup(lBottBack, lUpBack)
rearRightMG = MotorGroup(rBottBack, rUpBack)
frontLeftMG = MotorGroup(lBottFront, lUpFront)
frontRightMG = MotorGroup(rBottFront, rUpFront)
intakeMG = MotorGroup(intake, outFlap)
intakeMG.set_velocity(100, PERCENT)
outFlex.set_velocity(100, PERCENT)


#Controller Deadzone
DEADZONE = 10
POSITION_TOLERANCE = 2.0   # inches – how close is "arrived"
HEADING_TOLERANCE  = 5.0   # degrees
SETTLE_CYCLES      = 3     # must stay within tolerance this many loops (~60 ms)
TIMEOUT_MS         = 8000  # give up after this long

FIELD_ORIENTED = True


# ===== utils =====

def fieldToRobot(fieldX, fieldY, headingDeg):
    """Convert field-frame velocity to robot-frame using the given heading."""
    headingRad = headingDeg * (3.14159 / 180)
    robotX = fieldX * math.cos(headingRad) - fieldY * math.sin(headingRad)
    robotY = fieldX * math.sin(headingRad) + fieldY * math.cos(headingRad)
    return robotX, robotY


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def normalize_angle_deg(angle):
    """Wrap angle to the range (-180, 180]."""
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


def joystick_to_heading(axis1, axis2):
    """Convert right-joystick axes to a target heading in degrees (0 = forward/up, CW positive).
    axis1 = left/right (+right), axis2 = forward/back (+forward).
    Returns heading in 0-360 range matching IMU convention."""
    # atan2 gives angle from +X axis CCW; we want angle from +Y axis CW
    rad = math.atan2(axis1, axis2)
    deg = rad * (180 / 3.14159)
    # Normalize to 0-360
    if deg < 0:
        deg += 360
    return deg


# ===== pid =====

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


# ===== init =====

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
chassis_heading_pid = PIDController(kp=0.2, ki=0.0, kd=0.2, max_output=100)
driver_heading_pid = PIDController(kp=1.0, ki=0.0, kd=0.2, max_output=50)


# ===== inverse_kinematics =====

def XJoystickDrive(Xpos, Ypos, turn):
    if(abs(Xpos) > DEADZONE or abs(Ypos) > DEADZONE or abs(turn) > DEADZONE):
        if FIELD_ORIENTED:
            Xpos, Ypos, turn = fieldOrientedControl(Xpos, Ypos, turn)
        frontRightSpeed = Ypos - Xpos - turn
        rearRightSpeed = Ypos + Xpos - turn
        frontLeftSpeed = Ypos + Xpos + turn
        rearLeftSpeed = Ypos - Xpos + turn

        rearLeftMG.spin(FORWARD, rearLeftSpeed, PERCENT)
        rearRightMG.spin(FORWARD, rearRightSpeed, PERCENT)
        frontLeftMG.spin(FORWARD, frontLeftSpeed, PERCENT)
        frontRightMG.spin(FORWARD, frontRightSpeed, PERCENT)
    else:
        frontLeftMG.stop()
        frontRightMG.stop()
        rearLeftMG.stop()
        rearRightMG.stop()

def fieldOrientedControl(Xpos, Ypos, turn):
    heading = imu.heading()
    headingRad = heading * (3.14159 / 180)
    rotatedX = Xpos * math.cos(headingRad) - Ypos * math.sin(headingRad)
    rotatedY = Xpos * math.sin(headingRad) + Ypos * math.cos(headingRad)
    return rotatedX, rotatedY, turn


def driveToPoint(targetX, targetY, targetHeading, drive_state):
    """Run one PID tick toward the target. Returns True when arrived (or timed out).
    Call this every loop iteration; it initialises itself on the first call
    after the previous move finished."""

    # First call for a new target – reset PIDs and timers
    if not drive_state.initialized:
        drive_state.reset()

    # Read sensors once per tick
    currentX = gps.x_position()
    currentY = gps.y_position()
    currentHeading = gps.heading()

    # PID for each chassis axis (field frame)
    timestamp = brain.timer.time()
    lin_x = chassis_x_pid.calculate(targetX, currentX, timestamp)
    lin_y = chassis_y_pid.calculate(targetY, currentY, timestamp)
    heading_error = normalize_angle_deg(targetHeading - currentHeading)
    ang_z = chassis_heading_pid.calculate_error(heading_error, timestamp)

    # Field → robot frame
    robot_x, robot_y = fieldToRobot(lin_x, lin_y, currentHeading)

    # X-drive mixing: Fwd = robot_y, Stf = robot_x, Turn = ang_z (CW positive)
    front_right =  robot_y - robot_x - ang_z   # Fwd - Stf - Turn
    front_left  =  robot_y + robot_x + ang_z   # Fwd + Stf + Turn
    rear_right  =  robot_y + robot_x - ang_z   # Fwd + Stf - Turn
    rear_left   =  robot_y - robot_x + ang_z   # Fwd - Stf + Turn

    frontRightMG.spin(FORWARD, clamp(front_right, -100, 100), VelocityUnits.PERCENT)
    frontLeftMG.spin(FORWARD, clamp(front_left, -100, 100), VelocityUnits.PERCENT)
    rearRightMG.spin(FORWARD, clamp(rear_right, -100, 100), VelocityUnits.PERCENT)
    rearLeftMG.spin(FORWARD, clamp(rear_left, -100, 100), VelocityUnits.PERCENT)

    # Settle check
    dist_error = math.sqrt((targetX - currentX) ** 2 + (targetY - currentY) ** 2)
    if dist_error < POSITION_TOLERANCE and abs(heading_error) < HEADING_TOLERANCE:
        drive_state.settle_counter += 1
    else:
        drive_state.settle_counter = 0

    arrived = drive_state.settle_counter >= SETTLE_CYCLES
    timed_out = (timestamp - drive_state.start_time) > TIMEOUT_MS

    if arrived or timed_out:
        rUpFront.stop()
        lUpFront.stop()
        rBottBack.stop()
        lBottBack.stop()
        drive_state.initialized = False   # ready for next command
        return True

    return False


# ===== autonomous =====

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place autonomous code here


# ===== driver_control =====

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    dpadHeading = -1  # -1 = no d-pad heading active
    descoreOpen = False
    matchLoadOpen = False
    # place driver control in this while loop
    while True:
        # --- Heading control ---
        # D-pad snap-to-heading (overrides joystick while held)
        if controller.buttonUp.pressing():
            dpadHeading = 0
        elif controller.buttonDown.pressing():
            dpadHeading = 180
        elif controller.buttonLeft.pressing():
            dpadHeading = 270
        elif controller.buttonRight.pressing():
            dpadHeading = 90
        else:
            dpadHeading = -1

        # Right joystick sets target heading direction
        rightX = controller.axis1.position()  # left/right
        rightY = controller.axis2.position()  # forward/back
        rightMag = math.sqrt(rightX ** 2 + rightY ** 2)

        if dpadHeading >= 0:
            # D-pad heading snap takes priority
            currentHeading = imu.heading()
            headingError = normalize_angle_deg(dpadHeading - currentHeading)
            turn = driver_heading_pid.calculate_angle_error(headingError, brain.timer.time())
        elif rightMag > DEADZONE:
            targetHeading = joystick_to_heading(rightX, rightY)
            currentHeading = imu.heading()
            headingError = normalize_angle_deg(targetHeading - currentHeading)
            turn = driver_heading_pid.calculate_angle_error(headingError, brain.timer.time())
        else:
            turn = 0
            driver_heading_pid.reset()

        xPos = controller.axis4.position()
        yPos = controller.axis3.position()

        # --- Intake (bumpers) ---
        if controller.buttonR2.pressing() and controller.buttonR1.pressing():
            # R1+R2 combo: toggle match load
            if not matchLoadOpen:
                matchLoad.open()
                matchLoadOpen = True
            else:
                matchLoad.close()
                matchLoadOpen = False
            # debounce – wait for release
            while controller.buttonR2.pressing() and controller.buttonR1.pressing():
                wait(10, MSEC)
        elif controller.buttonR2.pressing():
            intakeMG.spin(FORWARD)
        elif controller.buttonR1.pressing():
            intakeMG.spin(REVERSE)
        else:
            intakeMG.stop()

        if controller.buttonL2.pressing():
            outFlex.spin(FORWARD)
        elif controller.buttonL1.pressing():
            outFlex.spin(REVERSE)
        else:
            outFlex.stop()

        # --- Face buttons ---
        # Y / X = heightMech
        if controller.buttonY.pressing():
            heightMech.open()
        elif controller.buttonX.pressing():
            heightMech.close()

        # B = descore toggle
        if controller.buttonB.pressing():
            if not descoreOpen:
                descore.open()
                descoreOpen = True
            else:
                descore.close()
                descoreOpen = False
            # debounce – wait for release
            while controller.buttonB.pressing():
                wait(10, MSEC)

        # A = reset IMU heading
        if controller.buttonA.pressing():
            imu.set_heading(0)
            driver_heading_pid.reset()
            # debounce – wait for release
            while controller.buttonA.pressing():
                wait(10, MSEC)

        XJoystickDrive(xPos, yPos, turn)
        wait(20, MSEC)


# ===== entry =====

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()
heightMech.close()
matchLoad.close()
descore.close()
