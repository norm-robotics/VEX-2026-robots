# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       anthony                                                      #
# 	Created:      2/21/2026, 2:28:33 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
import math

# Defines
brain = Brain()
gps = Gps(Ports.PORT12)
gps.calibrate()
gps.set_origin(5,6,INCHES)
USE_VISION_SENSOR=False
USE_OPTICAL_SENSOR=False
USE_FIELD_ORIENTED_CONTROL=False
USE_MESSAGE_LINK=False

# Conditionally initialise optional hardware
optical = None
if USE_OPTICAL_SENSOR:
    optical = Optical(Ports.PORT20)
    optical.set_light_power(100, PERCENT)
    optical.set_light(LedStateType.ON)

ai_camera = None
if USE_VISION_SENSOR:
    ai_camera = Vision(Ports.PORT19)

link = None
if USE_MESSAGE_LINK:
    link = MessageLink(Ports.PORT11, "49erRoboticsgoesbrrrrrr", VexlinkType.MANAGER)

# ---- Vision color signatures (tune these on real robot) ----
# Approximate HSV-based signatures for red & blue game elements
if USE_VISION_SENSOR and ai_camera is not None:
    RED_SIG   = Signature(1, 8000, 11000, 9500, -1500, -500, -1000, 3.0, 0)
    BLUE_SIG  = Signature(2, -3500, -1500, -2500, 5000, 9000, 7000, 3.0, 0)
else:
    RED_SIG  = None
    BLUE_SIG = None

# ---- Indexer state ----
# Tracks color of the game element currently in the intake path
# "red", "blue", or "none"
indexer_element_color = "none"
EJECT_REVERSE_MS = 400   # how long to reverse intake to eject wrong color

frontRightUpperMotor = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
frontRightLowerMotor = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
frontRightMotors = MotorGroup(frontRightUpperMotor,frontRightLowerMotor)
frontLeftUpperMotor = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
frontLeftLowerMotor = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
frontLeftMotors = MotorGroup(frontLeftUpperMotor,frontLeftLowerMotor)
rearRightUpperMotor = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
rearRightLowerMotor = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
rearRightMotors = MotorGroup(rearRightUpperMotor,rearRightLowerMotor)
rearLeftUpperMotor = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
rearLeftLowerMotor = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
rearLeftMotors = MotorGroup(rearLeftUpperMotor,rearLeftLowerMotor)
intakeMotorGround = Motor(Ports.PORT8, GearSetting.RATIO_6_1)
intakeMotorBig = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
intakeMotors = MotorGroup(intakeMotorGround,intakeMotorBig)
outtakeMotorTop = Motor(Ports.PORT21, GearSetting.RATIO_6_1)
outtakeMotors = MotorGroup(outtakeMotorTop)
matchLoadMech = Pneumatics(brain.three_wire_port.a)
heightMech = Pneumatics(brain.three_wire_port.b)
flap = Pneumatics(brain.three_wire_port.c)

controller = Controller()

# Team color: "red" or "blue" (set via touchscreen before match)
team_color = "red"

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

    def _step(self, error, timestamp):
        if self.last_time == 0:
            dt = 0.02
        else:
            dt = (timestamp - self.last_time) / 1000.0

        p_output = self.kp * error

        self.integral += error * dt
        # Clamp the integral contribution, not the raw accumulator
        i_output = self.ki * self.integral
        if i_output > self.max_output:
            i_output = self.max_output
            self.integral = self.max_output / self.ki if self.ki != 0 else 0
        elif i_output < -self.max_output:
            i_output = -self.max_output
            self.integral = -self.max_output / self.ki if self.ki != 0 else 0

        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0
        d_output = self.kd * derivative

        output = p_output + i_output + d_output
        output = max(min(output, self.max_output), -self.max_output)

        self.previous_error = error
        self.last_time = timestamp
        return output

    def calculate(self, target_value, current_value, timestamp):
        error = target_value - current_value
        return self._step(error, timestamp)

    def calculate_error(self, error, timestamp):
        return self._step(error, timestamp)

    def reset(self):
        self.integral = 0
        self.previous_error = 0
        self.last_time = 0


# PID controllers for chassis axes (lin x, lin y, ang z)
chassis_x_pid = PIDController(kp=0.5, ki=0.0, kd=0.00, max_output=100)
chassis_y_pid = PIDController(kp=0.5, ki=0.0, kd=0.00, max_output=100)
chassis_heading_pid = PIDController(kp=0.05, ki=0.0, kd=0.0, max_output=100)





# def autonomous():
#     gps.calibrate()
#     brain.screen.clear_screen()
#     # color = optical.rgb(False)
#     # invert = -1
#     # if color == (255, 0, 0):
#     #     invert = 1
#     # elif color == (0, 0, 255):
#     #     invert = -1
    
#     currentX = gps.x_position()
#     currentY = gps.y_position()
#     currentYaw = gps.heading()

def select_team_color():
    global team_color

    brain.screen.clear_screen()
    brain.screen.set_cursor(3, 5)
    brain.screen.print("Select team color:")
    brain.screen.set_cursor(5, 3)
    brain.screen.print("Left = RED   Right = BLUE")

    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
    controller.screen.print("LEFT=RED RIGHT=BLUE")

    # Wait for a button press on the controller
    while True:
        if controller.buttonLeft.pressing():
            team_color = "red"
            break
        elif controller.buttonRight.pressing():
            team_color = "blue"
            break
        wait(50, MSEC)

    # Confirm on both screens
    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
    controller.screen.print("Team: " + team_color.upper())

    brain.screen.clear_screen()
    if team_color == "red":
        brain.screen.set_fill_color(Color.RED)
    else:
        brain.screen.set_fill_color(Color.BLUE)
    brain.screen.set_pen_color(Color.WHITE)
    brain.screen.draw_rectangle(0, 0, 480, 240)
    brain.screen.set_cursor(5, 10)
    brain.screen.print("Team: " + team_color.upper())
    wait(800, MSEC)
    brain.screen.clear_screen()


def autonomous():
    gps.calibrate()
    brain.screen.clear_screen()

    step = 0
    wait_start = 0

    # Snapshot of position/heading – updated at each step transition
    snapX = gps.x_position()
    snapY = gps.y_position()
    snapYaw = gps.heading()
    while True:

        # Scan for game elements & publish positions each tick
        # scan_and_publish_elements()

        # # Check outtake color – eject if wrong alliance color
        # check_outtake_color()
        # if is_wrong_color():
        #     eject_wrong_element()

        # ---- step 0: drive forward 36 in ----
        if step == 0:
            if driveToPoint(snapX, snapY - 14, snapYaw):
                snapX = gps.x_position()
                snapY = gps.y_position()
                snapYaw = gps.heading()
                step = 1

        if step == 1:
            if driveToPoint(snapX, snapY, snapYaw+180):
                intakeMotors.spin(FORWARD, 100, VelocityUnits.PERCENT)
                matchLoadMech.open()
                wait_start = brain.timer.time()
                step = 2

        # ---- step 1: drive left 12 in ----
        elif step == 2:
            if driveToPoint(snapX - 6, snapY, snapYaw):
                wait_start = brain.timer.time()
                step = 3

        # ---- step 2: wait 1 s for intake ----
        elif step == 3:
            if brain.timer.time() - wait_start >= 1000:
                intakeMotors.stop()
                snapX = gps.x_position()
                snapY = gps.y_position()
                snapYaw = gps.heading()
                step = 4

        # ---- step 3: drive right 6 in ----
        elif step == 4:
            if driveToPoint(snapX + 6, snapY, snapYaw):
                matchLoadMech.close()
                heightMech.open()
                snapX = gps.x_position()
                snapY = gps.y_position()
                snapYaw = gps.heading()
                step = 5

        # ---- step 4: drive right 24 in, spin 180° ----
        elif step == 5:
            if driveToPoint(snapX + 24, snapY, snapYaw):
                intakeMotors.spin(FORWARD, 100, VelocityUnits.PERCENT)
                outtakeMotors.spin(FORWARD, 100, VelocityUnits.PERCENT)
                step = 6

        elif step == 6:
            if driveToPoint(snapX + 24, snapY, snapYaw + 180):
                intakeMotors.spin(FORWARD, 100, VelocityUnits.PERCENT)
                outtakeMotors.spin(FORWARD, 100, VelocityUnits.PERCENT)
                wait_start = brain.timer.time()
                step = 7

        # ---- step 5: wait 1 s for outtake ----
        elif step == 7:
            if brain.timer.time() - wait_start >= 1000:
                outtakeMotors.stop()
                snapX = gps.x_position()
                snapY = gps.y_position()
                snapYaw = gps.heading()
                step = 8

        # ---- step 6: drive forward 12 in, turn -90° ----
        elif step == 8:
            if driveToPoint(snapX + 12, snapY, snapYaw - 90):
                heightMech.close()
                wait_start = brain.timer.time()
                step = 9

        # ---- step 7: wait 500 ms ----
        elif step == 9:
            if brain.timer.time() - wait_start >= 500:
                snapX = gps.x_position()
                snapY = gps.y_position()
                snapYaw = gps.heading()
                step = 10
                intakeMotors.stop()
                outtakeMotors.stop()    

        # ---- step 8: drive right 96 in ----
        # elif step == 8:
        #     if driveToPoint(snapX + 96, snapY, snapYaw):
        #         intakeMotors.stop()
        #         heightMech.open()
        #         snapX = gps.x_position()
        #         snapY = gps.y_position()
        #         snapYaw = gps.heading()
        #         step = 9

        # # ---- step 9: drive left 12 in, forward 12 in, turn +90° ----
        # elif step == 9:
        #     if driveToPoint(snapX - 12, snapY- 12, snapYaw + 90):
        #         intakeMotors.spin(FORWARD, 100, VelocityUnits.PERCENT)
        #         outtakeMotors.spin(FORWARD, 100, VelocityUnits.PERCENT)
        #         wait_start = brain.timer.time()
        #         step = 10

        # # ---- step 10: wait 1 s for outtake ----
        # elif step == 10:
        #     if brain.timer.time() - wait_start >= 1000:
        #         outtakeMotors.stop()
        #         intakeMotors.stop()
        #         snapX = gps.x_position()
        #         snapY = gps.y_position()
        #         snapYaw = gps.heading()
        #         step = 11

        # # ---- step 11: drive right 12 in, turn 180° ----
        # elif step == 11:
        #     if driveToPoint(snapX + 12, snapY, snapYaw + 180):
        #         step = 12

        # # ---- done ----
        # elif step == 12:
        #     brain.screen.print("autonomous done")
        #     break

        wait(20, MSEC)

def user_control():
    gps.calibrate()
    select_team_color()

    brain.screen.clear_screen()
    # color = optical.color()
    brain.screen.print("driver control")

    # Toggle states for pistons
    heightMech_on = False
    flap_on = False
    matchLoadMech_on = False

    # Previous button states for edge detection
    prev_up = False
    prev_left = False
    prev_right = False

    # Sensitivity: 0.0–1.0 (lower = slower / less sensitive)
    DRIVE_SENSITIVITY = 0.5
    TURN_SENSITIVITY  = 0.4

    # place driver control in this while loop
    while True:
        # Cubic curve: (axis/100)^3 * 100 * sensitivity → smoother fine control
        raw_rot = controller.axis1.position()
        raw_y   = controller.axis3.position()
        raw_x   = controller.axis4.position()
        rotation = -(raw_rot / 100.0) ** 3 * 100.0 * TURN_SENSITIVITY  if raw_rot != 0 else 0  # right stick X
        Ypos     = -(raw_y   / 100.0) ** 3 * 100.0 * DRIVE_SENSITIVITY if raw_y   != 0 else 0  # left stick Y
        Xpos     = -(raw_x   / 100.0) ** 3 * 100.0 * DRIVE_SENSITIVITY if raw_x   != 0 else 0  # left stick X
        # ---- Intake with color indexing ----
        if controller.buttonR1.pressing():
            # Check intake color before running forward
            check_intake_color()
            if is_wrong_color():
                eject_wrong_element()
            else:
                intakeMotors.spin(FORWARD, 100, VelocityUnits.PERCENT)
        elif controller.buttonR2.pressing():
            intakeMotors.spin(REVERSE, 100, VelocityUnits.PERCENT)
        else:
            intakeMotors.stop()

        # ---- Outtake with optical color gate ----
        if controller.buttonL1.pressing():
            check_outtake_color()
            if is_wrong_color():
                eject_wrong_element()
            else:
                outtakeMotors.spin(FORWARD, 100, VelocityUnits.PERCENT)
        elif controller.buttonL2.pressing():
            outtakeMotors.spin(REVERSE, 100, VelocityUnits.PERCENT)
        else:
            outtakeMotors.stop()

        # Periodic vision scan & MessageLink broadcast
        scan_and_publish_elements()
        
        # Sticky toggle for heightMech (buttonUp)
        cur_up = controller.buttonUp.pressing()
        if cur_up and not prev_up:
            heightMech_on = not heightMech_on
        prev_up = cur_up
        if heightMech_on:
            heightMech.open()
        else:
            heightMech.close()

        # Sticky toggle for flap (buttonLeft)
        cur_left = controller.buttonLeft.pressing()
        if cur_left and not prev_left:
            flap_on = not flap_on
        prev_left = cur_left
        if flap_on:
            flap.open()
        else:
            flap.close()

        # Sticky toggle for matchLoadMech (buttonRight)
        cur_right = controller.buttonRight.pressing()
        if cur_right and not prev_right:
            matchLoadMech_on = not matchLoadMech_on
        prev_right = cur_right
        if matchLoadMech_on:
            matchLoadMech.open()
        else:
            matchLoadMech.close()
  
        XDriveJoystick(Ypos, Xpos, rotation)
        wait(20, MSEC)


def XDriveJoystick(Xpos, Ypos, turn):
    if USE_FIELD_ORIENTED_CONTROL:
        rotatedX, rotatedY, turn = fieldOrientedControl(Xpos, Ypos, turn)
    else:
        rotatedX = Xpos
        rotatedY = Ypos

    frontRightMotorMove = rotatedY - rotatedX - turn
    frontLeftMotorMove = -rotatedY - rotatedX + turn
    rearRightMotorMove = rotatedY + rotatedX + turn
    rearLeftMotorMove = -rotatedY + rotatedX - turn

    frontRightMotors.spin(FORWARD, frontRightMotorMove, VelocityUnits.PERCENT)
    frontLeftMotors.spin(FORWARD, frontLeftMotorMove, VelocityUnits.PERCENT)
    rearRightMotors.spin(FORWARD, rearRightMotorMove, VelocityUnits.PERCENT)
    rearLeftMotors.spin(FORWARD, rearLeftMotorMove, VelocityUnits.PERCENT)
    



def fieldOrientedControl(Xpos, Ypos, turn):
    heading = gps.heading()
    headingRad = heading * (3.14159 / 180)
    rotatedX = Xpos * math.cos(headingRad) - Ypos * math.sin(headingRad)
    rotatedY = Xpos * math.sin(headingRad) + Ypos * math.cos(headingRad)
    return rotatedX, rotatedY, turn


# ---- Vision: scan field & publish game-element positions over MessageLink ----
def scan_and_publish_elements():
    """Take snapshots for red & blue elements, publish field coords via link."""
    if not USE_VISION_SENSOR or ai_camera is None:
        return

    robot_x = gps.x_position()
    robot_y = gps.y_position()
    robot_heading = gps.heading()

    for sig, color_name in [(RED_SIG, "red"), (BLUE_SIG, "blue")]:
        if sig is None:
            continue
        objects = ai_camera.take_snapshot(AiVision.ALL_AIOBJS)
        if objects is None:
            continue
        for obj in objects:
            # Convert vision pixel offset to rough field-relative inches
            # Vision centre ~158 px, FOV ~61°, rough px-to-angle conversion
            angle_offset = (obj.centerX - 158) * (61.0 / 316.0)
            element_heading = robot_heading + angle_offset
            # Rough distance estimate from object height (bigger = closer)
            est_dist = 2000.0 / max(obj.height, 1)
            rad = element_heading * (3.14159 / 180)
            field_x = robot_x + est_dist * math.sin(rad)
            field_y = robot_y + est_dist * math.cos(rad)

            if USE_MESSAGE_LINK and link is not None:
                if link.is_linked():
                    link.send("elem:" + color_name + "," +
                            str(round(field_x, 1)) + "," +
                            str(round(field_y, 1)))
                    
                else:
                    brain.screen.set_cursor(1, 1)
                    brain.screen.print("Link not connected")

    # Also broadcast our alliance color
    if USE_MESSAGE_LINK and link is not None:
        if link.is_linked():
            link.send("alliance:" + team_color)
        else:
            brain.screen.set_cursor(1, 1)
            brain.screen.print("Link not connected")


# ---- Intake indexer helpers ----
def check_intake_color():
    """Use AI vision camera to detect the color of the game element being intaked.
    Returns 'red', 'blue', or 'none'."""
    global indexer_element_color
    if not USE_VISION_SENSOR or ai_camera is None:
        return "none"

    # Check red first
    if RED_SIG is not None:
        objs = ai_camera.take_snapshot(RED_SIG)
        if objs is not None and len(objs) > 0:
            # Is the largest object close / centred (i.e. in the intake)?
            if objs[0].width > 40 and abs(objs[0].centerX - 158) < 60:
                indexer_element_color = "red"
                return "red"

    if BLUE_SIG is not None:
        objs = ai_camera.take_snapshot(BLUE_SIG)
        if objs is not None and len(objs) > 0:
            if objs[0].width > 40 and abs(objs[0].centerX - 158) < 60:
                indexer_element_color = "blue"
                return "blue"

    indexer_element_color = "none"
    return "none"


def check_outtake_color():
    """Use optical sensor to read the color of the element just before outtake.
    Returns 'red', 'blue', or 'none'."""
    global indexer_element_color
    if not USE_OPTICAL_SENSOR or optical is None:
        return indexer_element_color   # fall back to vision reading

    hue = optical.hue()
    # Red hue wraps around 0/360: roughly 0-30 or 330-360
    if hue < 30 or hue > 330:
        indexer_element_color = "red"
        return "red"
    # Blue hue: roughly 200-260
    elif 200 <= hue <= 260:
        indexer_element_color = "blue"
        return "blue"

    return indexer_element_color   # keep previous reading if ambiguous


def is_wrong_color():
    """Returns True if the element currently in the path is the opposing alliance color."""
    if indexer_element_color == "none":
        return False
    if team_color == "red" and indexer_element_color == "blue":
        return True
    if team_color == "blue" and indexer_element_color == "red":
        return True
    return False


def eject_wrong_element():
    """Reverse intake briefly to throw out the wrong-color element."""
    global indexer_element_color
    intakeMotors.spin(REVERSE, 100, VelocityUnits.PERCENT)
    wait(EJECT_REVERSE_MS, MSEC)
    intakeMotors.stop()
    indexer_element_color = "none"


def fieldToRobot(fieldX, fieldY, headingDeg):
    """Convert field-frame velocity to robot-frame using the given heading."""
    headingRad = headingDeg * (3.14159 / 180)
    robotX = fieldX * math.cos(headingRad) - fieldY * math.sin(headingRad)
    robotY = fieldX * math.sin(headingRad) + fieldY * math.cos(headingRad)
    return robotX, robotY


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def normalize_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


POSITION_TOLERANCE = 1.5   # inches – how close is "arrived"
HEADING_TOLERANCE  = 3.0   # degrees
SETTLE_CYCLES      = 5     # must stay within tolerance this many loops (~100 ms)
TIMEOUT_MS         = 5000  # give up after this long


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


def driveToPoint(targetX, targetY, targetHeading):
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

    # X-drive mixing
    front_right = robot_y - robot_x - ang_z
    front_left  = -robot_y - robot_x + ang_z
    rear_right  = robot_y + robot_x + ang_z
    rear_left   = -robot_y + robot_x - ang_z

    frontRightMotors.spin(FORWARD, clamp(front_right, -100, 100), VelocityUnits.PERCENT)
    frontLeftMotors.spin(FORWARD, clamp(front_left, -100, 100), VelocityUnits.PERCENT)
    rearRightMotors.spin(FORWARD, clamp(rear_right, -100, 100), VelocityUnits.PERCENT)
    rearLeftMotors.spin(FORWARD, clamp(rear_left, -100, 100), VelocityUnits.PERCENT)

    # Settle check
    dist_error = math.sqrt((targetX - currentX) ** 2 + (targetY - currentY) ** 2)
    if dist_error < POSITION_TOLERANCE and abs(heading_error) < HEADING_TOLERANCE:
        drive_state.settle_counter += 1
    else:
        drive_state.settle_counter = 0

    arrived = drive_state.settle_counter >= SETTLE_CYCLES
    timed_out = (timestamp - drive_state.start_time) > TIMEOUT_MS

    if arrived or timed_out:
        frontRightMotors.stop()
        frontLeftMotors.stop()
        rearRightMotors.stop()
        rearLeftMotors.stop()
        drive_state.initialized = False   # ready for next command
        return True

    return False

# ---- Pre-match team color selection via Controller ----


comp = Competition(user_control, autonomous)

