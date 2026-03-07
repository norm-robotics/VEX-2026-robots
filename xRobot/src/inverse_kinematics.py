from config import * # lsp-only
from utils import * # lsp-only
from init import * # lsp-only
from sensor_fusion import * # lsp-only

def XJoystickDrive(Xpos, Ypos, turn):
    if(abs(Xpos) > DEADZONE or abs(Ypos) > DEADZONE or abs(turn) > DEADZONE):
        if FIELD_ORIENTED:
            Xpos, Ypos, turn = fieldOrientedControl(Xpos, Ypos, turn)
        frontRightSpeed = clamp(Ypos - Xpos - turn, -50, 50)
        rearRightSpeed = clamp(Ypos + Xpos - turn, -50, 50)
        frontLeftSpeed = clamp(Ypos + Xpos + turn, -50, 50)
        rearLeftSpeed = clamp(Ypos - Xpos + turn, -50, 50)

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
    heading = fusion.get_heading()
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

    # Update sensor fusion and read fused state
    fusion.update()
    currentX, currentY = fusion.get_position()
    currentHeading = fusion.get_heading()

    # PID for each chassis axis (field frame)
    timestamp = brain.timer.time()
    lin_x = chassis_x_pid.calculate(targetX, currentX, timestamp)
    lin_y = chassis_y_pid.calculate(targetY, currentY, timestamp)
    heading_error = normalize_angle_deg(targetHeading - currentHeading)
    ang_z = chassis_heading_pid.calculate_error(heading_error, timestamp)

    # Pass field-frame PID outputs to XJoystickDrive.
    # fieldOrientedControl inside XJoystickDrive handles field→robot conversion.
    XJoystickDrive(lin_x, lin_y, ang_z)

    # Settle check
    dist_error = math.sqrt((targetX - currentX) ** 2 + (targetY - currentY) ** 2)
    if dist_error < POSITION_TOLERANCE and abs(heading_error) < HEADING_TOLERANCE:
        drive_state.settle_counter += 1
    else:
        drive_state.settle_counter = 0

    arrived = drive_state.settle_counter >= SETTLE_CYCLES
    timed_out = (timestamp - drive_state.start_time) > TIMEOUT_MS

    if arrived or timed_out:
        XJoystickDrive(0, 0, 0)  # stop the robot
        drive_state.initialized = False   # ready for next command
        return True

    return False
