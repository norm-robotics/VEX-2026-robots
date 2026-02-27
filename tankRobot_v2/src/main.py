# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       govind                                                       #
# 	Created:      2/26/2026, 3:09:09 PM                                        #
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
lBottom = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
rBottom = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)

#Upper DriveTrain Motors
lUpFront = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
lUpBack = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
rUpFront = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
rUpBack = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)

#Intake Motors
intake = Motor(Ports.PORT17, GearSetting.RATIO_6_1, False)

#Outtake Motors
outFlex = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False)
outFlap = Motor(Ports.PORT15, GearSetting.RATIO_18_1, False)

#sensor Motors
gps = Gps(Ports.PORT20)

#Pneumatics
heightMech = Pneumatics(brain.three_wire_port.a)
flap = Pneumatics(brain.three_wire_port.b)

#MotorGroups (MG)
leftMG = MotorGroup(lBottom, lUpBack, lUpFront)
rightMG = MotorGroup(rBottom, rUpBack, rUpFront)
intakeMG = MotorGroup(intake, outFlap)


#Calling required constructors
drivetrain = DriveTrain(leftMG, rightMG)

def autonomous():
    pass

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("DRIVER MODE =(")
    
    #Defining max speeds for moving and turning

    while True:
    #-----------------------------------------------
        #Defining buttons on the controller
    #-----------------------------------------------

        #Intake and outtake controls
        if controller.buttonR1.pressing():
            intakeMG.spin(FORWARD, 100, PERCENT)
        elif controller.buttonR2.pressing():
            intakeMG.spin(REVERSE, 100, PERCENT)
        else:
            intakeMG.stop()

        if controller.buttonL1.pressing():
            outFlex.spin(FORWARD, 100, PERCENT)
        elif controller.buttonL2.pressing():
            outFlex.spin(REVERSE, 100, PERCENT)
        else:
            outFlex.stop()

        #pneumatic controls
        if controller.buttonUp.pressing():
            heightMech.open()
        elif controller.buttonDown.pressing():
            heightMech.close()

        turn = 0.65*controller.axis1.position()
        drive = 0.85*controller.axis3.position()

        leftSpeed = drive + turn
        rightSpeed = drive - turn

        leftDead = max(-100, min(100, leftSpeed))
        rightDead = max(-100, min(100, rightSpeed))

        leftMG.set_velocity((leftSpeed), PERCENT)
        rightMG.set_velocity((rightSpeed), PERCENT)

        if (abs(leftDead) < 2) or (abs(rightDead) < 2):
            drivetrain.stop()
        else:
            drivetrain.drive(FORWARD)
        
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()
    