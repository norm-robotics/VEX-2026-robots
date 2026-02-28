# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       govind                                                       #
# 	Created:      2/27/2026, 11:10:03 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       govind                                                       #
# 	Created:      2/27/2026, 2:12:16 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain and controller should be defined by default
brain=Brain()
controller = Controller()

#================================
#DEFINING MOTORS
#================================
#Bottom DriveTrain Motors
lBottFront = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
lBottBack = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
rBottFront = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
rBottBack = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)

#Upper DriveTrain Motors
lUpFront = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
lUpBack = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
rUpFront = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
rUpBack = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)

#Intake Motors
intake = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)

#Outtake Motors
outFlex = Motor(Ports.PORT21, GearSetting.RATIO_6_1, False)
outFlap = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)

#sensor Motors
gps = Gps(Ports.PORT20)

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


#Controller Deadzone
deadZone = 10
def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    # place driver control in this while loop
    while True:
        turn = controller.axis1.position()  
        xPos = controller.axis3.position()  
        yPos = controller.axis4.position()
        if(controller.buttonR2.pressing()):
            intakeMG.spin(FORWARD)
        elif(controller.buttonR1.pressing()):
            intakeMG.spin(REVERSE)
        else:
            intakeMG.stop()
        if(controller.buttonL2.pressing()):
            outFlex.spin(FORWARD)
        elif(controller.buttonL1.pressing()):
            outFlex.spin(REVERSE)
        else:
            outFlex.stop()
        
        if(controller.buttonUp.pressing()):
            heightMech.open()
        elif(controller.buttonDown.pressing()):
            heightMech.close()

        if(controller.buttonX.pressing()):
            matchLoad.open()
        elif(controller.buttonB.pressing()):
            matchLoad.close()

        if(controller.buttonLeft.pressing()):
            descore.open()
        elif(controller.buttonRight.pressing()):
            descore.close()

        XJoystickDrive(xPos, yPos, turn)
        wait(20, MSEC)
def XJoystickDrive(Xpos, Ypos, turn):
    if(abs(Xpos) > deadZone or abs(Ypos) > deadZone or abs(turn) > deadZone):
        frontRightSpeed = Ypos - Xpos - turn
        rearRightSpeed = Ypos + Xpos - turn
        frontLeftSpeed = -Ypos - Xpos - turn
        rearLeftSpeed = -Ypos + Xpos - turn

        rearLeftMG.spin(FORWARD, rearLeftSpeed, PERCENT)
        rearRightMG.spin(FORWARD, rearRightSpeed, PERCENT)
        frontLeftMG.spin(FORWARD, frontLeftSpeed, PERCENT)
        frontRightMG.spin(FORWARD, frontRightSpeed, PERCENT)
    else:
        frontLeftMG.stop()
        frontRightMG.stop()
        rearLeftMG.stop()
        rearRightMG.stop()

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()
heightMech.close()
matchLoad.close()
descore.close()