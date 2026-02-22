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

# Defines
brain = Brain()
gps = Gps(Ports.PORT12)
frontRightUpperMotor = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
frontRightLowerMotor = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
frontRightMotors = MotorGroup(frontRightUpperMotor,frontRightLowerMotor)
frontLeftUpperMotor = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
frontLeftLowerMotor = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)
frontLeftMotors = MotorGroup(frontLeftUpperMotor,frontLeftLowerMotor)
rearRightUpperMotor = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
rearRightLowerMotor = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
rearRightMotors = MotorGroup(rearRightUpperMotor,rearRightLowerMotor)
rearLeftUpperMotor = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
rearLeftLowerMotor = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)
rearLeftMotors = MotorGroup(rearLeftUpperMotor,rearLeftLowerMotor)
intakeMotorGround = Motor(Ports.PORTFIXEME, GearSetting.RATIO_6_1)
intakeMotorBig = Motor(Ports.PORTFIXEME, GearSetting.RATIO_18_1)
intakeMotors = MotorGroup(intakeMotorGround,intakeMotorBig)
outtakeMotorGround = Motor(Ports.PORTFIXEME, GearSetting.RATIO_6_1)
outtakeMotorBig = Motor(Ports.PORTFIXEME, GearSetting.RATIO_6_1)
outtakeMotors = MotorGroup(outtakeMotorGround,outtakeMotorBig)
matchLoadMech = Pneumatics(brain.three_wire_port.a)
heightMech = Pneumatics(brain.three_wire_port.b)
flap = Pneumatics(brain.three_wire_port.c)



def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    controller = Controller()

    # place driver control in this while loop
    while True:
        rotation = controller.axis2.position()
        Ypos = -controller.axis3.position()
        Xpos = -controller.axis4.position()
        if controller.buttonR1.pressing():
            intakeMotors.spin(FORWARD)
        else:
            if controller.buttonR2.pressing():
                intakeMotors.spin(REVERSE)
            else:
                intakeMotors.stop()
        if controller.buttonL1.pressing():
            outtakeMotors.spin(FORWARD)
        else:
            if controller.buttonL2.pressing():
                outtakeMotors.spin(REVERSE)
            else:
                outtakeMotors.stop()
        if controller.buttonUp.pressing():
            heightMech.open()
        else:
            heightMech.close()
        if controller.buttonLeft.pressing():
            flap.open()
        else:
            flap.close()
        if controller.buttonRight.pressing():
            matchLoadMech.open()
        else:
            matchLoadMech.close()
        XDriveJoystick(Ypos, Xpos, rotation)
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()

def XDriveJoystick(Xpos, Ypos, turn):
    frontRightMotorMove = Ypos - Xpos - turn
    frontLeftMotorMove = -Ypos - Xpos - turn
    rearRightMotorMove = Ypos + Xpos - turn
    rearLeftMotorMove = -Ypos + Xpos - turn
    if(frontLeftMotorMove > 0):
        frontLeftMotors.spin(FORWARD, frontLeftMotorMove, VelocityUnits.PERCENT)
    else:
        if(frontLeftMotorMove < 0):
            frontLeftMotors.spin(REVERSE, frontLeftMotorMove, VelocityUnits.PERCENT)
        else:
            frontLeftMotors.stop()
    if(frontRightMotorMove > 0):
        frontRightMotors.spin(FORWARD, frontLeftMotorMove, VelocityUnits.PERCENT)
    else:
        if(frontRightMotorMove < 0):
            frontRightMotors.spin(REVERSE, frontLeftMotorMove, VelocityUnits.PERCENT)
        else:
            frontRightMotors.stop()
    if(rearLeftMotorMove > 0):
        rearLeftMotors.spin(FORWARD, frontLeftMotorMove, VelocityUnits.PERCENT)
    else:
        if(rearLeftMotorMove < 0):
            rearLeftMotors.spin(REVERSE, frontLeftMotorMove, VelocityUnits.PERCENT)
        else:
            rearLeftMotors.stop()
    if(rearRightMotorMove > 0):
        rearRightMotors.spin(FORWARD, frontLeftMotorMove, VelocityUnits.PERCENT)
    else:
        if(rearRightMotorMove < 0):
            rearRightMotors.spin(REVERSE, frontLeftMotorMove, VelocityUnits.PERCENT)
        else:
            rearRightMotors.stop()