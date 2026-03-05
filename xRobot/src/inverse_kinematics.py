from config import * # lsp-only
from utils import * # lsp-only
from init import * # lsp-only

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
