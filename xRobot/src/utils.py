
import math
from config import * # lsp-only

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