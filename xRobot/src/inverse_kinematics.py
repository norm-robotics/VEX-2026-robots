from config import * # lsp-only

def XJoystickDrive(Xpos, Ypos, turn):
    if(abs(Xpos) > deadZone or abs(Ypos) > deadZone or abs(turn) > deadZone):
        frontRightSpeed = Ypos - Xpos - turn
        rearRightSpeed = Ypos - Xpos + turn
        frontLeftSpeed = Ypos + Xpos + turn
        rearLeftSpeed = Ypos + Xpos - turn

        rearLeftMG.spin(FORWARD, rearLeftSpeed, PERCENT)
        rearRightMG.spin(FORWARD, rearRightSpeed, PERCENT)
        frontLeftMG.spin(FORWARD, frontLeftSpeed, PERCENT)
        frontRightMG.spin(FORWARD, frontRightSpeed, PERCENT)
    else:
        frontLeftMG.stop()
        frontRightMG.stop()
        rearLeftMG.stop()
        rearRightMG.stop()