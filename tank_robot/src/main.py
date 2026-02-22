# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       anthony                                                      #
# 	Created:      2/21/2026, 7:24:08 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

brain = Brain()
#FIXME: Ports are not correct
leftTopFrontMotor = Motor(Ports.PORT3, True)
leftTopRearMotor = Motor(Ports.PORT10, True)
leftLowerMotor = Motor(Ports.PORT4, True)
leftMotorGroup = MotorGroup(leftLowerMotor, leftTopFrontMotor, leftTopRearMotor)
rightTopFrontMotor = Motor(Ports.PORT1, True)
rightTopRearMotor = Motor(Ports.PORT19, True)
rightLowerMotor = Motor(Ports.PORT2, True)
rightMotorGroup = MotorGroup(rightLowerMotor, rightTopFrontMotor, rightTopRearMotor)
outtakeFrontMotor = Motor(Ports.PORT18, False)
outtakeRearMotor = Motor(Ports.PORT15, False)
intakeMotor = Motor(Ports.PORT17, False)
intakeMotors = MotorGroup(outtakeRearMotor, intakeMotor)
gps = Gps(Ports.PORT20)
drivetrain = SmartDrive(leftMotorGroup, rightMotorGroup, gps, 9.709, 8, 11.1, DistanceUnits.IN, 37/63)
heightMech = Pneumatics(brain.three_wire_port.a)
flap = Pneumatics(brain.three_wire_port.b)

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here, to pick up the balls on the floor

    # Front starts at 15 inches from wall, drive another 12 inches
    drivetrain.drive_for(FORWARD, 12, INCHES)
    # Turn 90 degrees to the right
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    # Try to line up with the match loader, and spin intake
    intakeMotors.spin(FORWARD)
    drivetrain.drive_for(FORWARD, 48, INCHES)
    wait(1, SECONDS)
    intakeMotors.stop()
    # Reverse, turn to the right, move under the tube, get the 4 under there
    drivetrain.drive_for(REVERSE, 6, INCHES)
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 36, INCHES)
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 18, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    intakeMotors.spin(FORWARD)
    drivetrain.drive_for(FORWARD, 24, INCHES)
    wait(1, SECONDS)
    intakeMotors.stop()
    # Try to get to the bottom tube to unload
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 12, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 24, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 12, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 12, INCHES)
    heightMech.open()
    outtakeFrontMotor.spin(FORWARD)
    outtakeRearMotor.spin(FORWARD)
    wait(5, SECONDS)
    outtakeFrontMotor.stop()
    outtakeRearMotor.stop()
    heightMech.close()
    # Pick up the 6 game elements in the middle bottom, and deposite it in the lower tube
    drivetrain.drive_for(REVERSE, 12, INCHES)
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 12, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    intakeMotors.spin(FORWARD)
    drivetrain.drive_for(FORWARD, 36, INCHES)
    wait(1, SECONDS)
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 12, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 12, INCHES)
    drivetrain.turn_for(RIGHT, 45, DEGREES)
    drivetrain.drive_for(FORWARD, 20, INCHES)
    outtakeFrontMotor.spin(FORWARD)
    outtakeRearMotor.spin(FORWARD)
    wait(5, SECONDS)
    outtakeFrontMotor.stop()
    outtakeRearMotor.stop()
    # Reverse, go up, try to get the 4 under the tube, score those, then get the 6 in the middle, and score those in the middle tube
    drivetrain.drive_for(REVERSE, 18, INCHES)
    drivetrain.turn_for(LEFT, 45, DEGREES)
    drivetrain.drive_for(FORWARD, 48, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 12, INCHES)
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 24, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    intakeMotors.spin(FORWARD)
    drivetrain.drive_for(FORWARD, 24, INCHES)
    wait(1, SECONDS)
    intakeMotors.stop()
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 12, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 24, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 12, INCHES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    heightMech.open()
    drivetrain.drive_for(FORWARD, 12, INCHES)
    outtakeFrontMotor.spin(FORWARD)
    outtakeRearMotor.spin(FORWARD)
    wait(5, SECONDS)
    outtakeFrontMotor.stop()
    outtakeRearMotor.stop()
    drivetrain.drive_for(REVERSE, 12, INCHES)
    heightMech.close()
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 12, INCHES)
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 36, INCHES)
    drivetrain.turn_for(LEFT, 90, DEGREES)
    intakeMotors.spin(FORWARD)
    drivetrain.drive_for(FORWARD, 36, INCHES)
    wait(1, SECONDS)
    intakeMotors.stop()
    drivetrain.turn_for(RIGHT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 24, INCHES)
    drivetrain.turn_for(RIGHT, 135, DEGREES)
    heightMech.open()
    drivetrain.drive_for(FORWARD, 20, INCHES)
    outtakeFrontMotor.spin(FORWARD)
    outtakeRearMotor.spin(FORWARD)
    wait(5, SECONDS)
    outtakeFrontMotor.stop()
    outtakeRearMotor.stop()
    drivetrain.drive_for(REVERSE, 20, INCHES)
    heightMech.close()
    # Go to parking
    drivetrain.turn_for(RIGHT, 135, DEGREES)
    drivetrain.drive_for(FORWARD, 24, INCHES)
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 24, INCHES)
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 24, INCHES)
    drivetrain.stop()
    

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    controller = Controller()
    # place driver control in this while loop
    while True:
        turn = 0.85*controller.axis3.position() #0.85 is the for trim value from original code
        drive = -0.6*controller.axis4.position() #0.6 is the lat trim value from original code
        if controller.buttonR1.pressing():
            intakeMotors.spin(FORWARD)
        else:
            if controller.buttonR2.pressing():
                intakeMotors.spin(REVERSE)
            else:
                intakeMotors.stop()
        if controller.buttonL1.pressing():
            outtakeFrontMotor.spin(FORWARD)
        else:
            if controller.buttonL2.pressing():
                outtakeFrontMotor.spin(REVERSE)
            else:
                outtakeFrontMotor.stop()
        controller.buttonUp.pressed(heightMech.open)
        controller.buttonDown.pressed(heightMech.close)
        controller.buttonLeft.pressed(flap.open)
        controller.buttonRight.pressed(flap.close)
        if(drive >= 3):
            drivetrain.drive(FORWARD)
        else: 
            if(drive <= -3):
                drivetrain.drive(REVERSE)
            else:   
                drivetrain.stop()
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()
