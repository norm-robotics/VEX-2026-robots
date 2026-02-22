# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       anthony                                                      #
# 	Created:      2/21/2026, 7:24:08 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
# from concurrent.futures import wait

from vex import *

brain = Brain()
leftTopFrontMotor = Motor(Ports.PORT3, True)
leftTopRearMotor = Motor(Ports.PORT10, True)
leftLowerMotor = Motor(Ports.PORT4, True)
leftMotorGroup = MotorGroup(leftLowerMotor, leftTopFrontMotor, leftTopRearMotor)
rightTopFrontMotor = Motor(Ports.PORT1, False)
rightTopRearMotor = Motor(Ports.PORT19, False)
rightLowerMotor = Motor(Ports.PORT2, False)
rightMotorGroup = MotorGroup(rightLowerMotor, rightTopFrontMotor, rightTopRearMotor)
outtakeFrontMotor = Motor(Ports.PORT18, False)
outtakeRearMotor = Motor(Ports.PORT15, False)
intakeMotor = Motor(Ports.PORT17, False)
intakeMotors = MotorGroup(outtakeRearMotor, intakeMotor)
gps = Gps(Ports.PORT20)
inertial = Inertial(Ports.PORT11)
drivetrain = SmartDrive(leftMotorGroup, rightMotorGroup, inertial, 9.709, 8, 11.1, DistanceUnits.IN, 37/63)
drivetrain2 = Drivetrain(leftMotorGroup, rightMotorGroup)   
heightMech = Pneumatics(brain.three_wire_port.a)
# flap = Pneumatics(brain.three_wire_port.b)

# def autonomous():
#     brain.screen.clear_screen()
#     brain.screen.print("autonomous code")

#     inertial.calibrate()
#     while inertial.is_calibrating():
#         wait(50, MSEC)
#     # place automonous code here, to pick up the balls on the floor
#     gps.calibrate()
#     # Front starts at 15 inches from wall, drive another 12 inches
#     drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
#     # Turn 90 degrees to the right
#     drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
#     # Try to line up with the match loader, and spin intake
#     intakeMotors.spin(FORWARD)
#     drivetrain.drive_for(FORWARD, 52, INCHES, wait=True)
#     wait(1, SECONDS)
#     intakeMotors.stop()
#     # Reverse, turn to the right, move under the tube, get the 4 under there
#     drivetrain.drive_for(REVERSE, 6, INCHES, wait=True)
#     drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
#     drivetrain.drive_for(FORWARD, 36, INCHES, wait=True)
#     drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
#     drivetrain.drive_for(FORWARD, 18, INCHES, wait=True)
#     drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
#     intakeMotors.spin(FORWARD)
#     drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
#     wait(1, SECONDS)
#     intakeMotors.stop()
#     # Try to get to the bottom tube to unload
#     drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
#     drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
#     drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
#     drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
#     drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
#     drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
#     drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
#     # heightMech.open()
#     # drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
#     # outtakeFrontMotor.spin(FORWARD)
#     # outtakeRearMotor.spin(FORWARD)
#     # wait(5, SECONDS)
#     # outtakeFrontMotor.stop()
#     # outtakeRearMotor.stop()
#     # heightMech.close()utonomous():
#     brain.screen.clear_screen()
#     brain.screen.print("autonomous code")

#     inertial.calibrate()
#     while inertial.is_calibrating():
#         wait(50, MSEC)
#     # place automonous code here, to pick up the balls on the floor
#     gps.calibrate()
#     # Front starts at 15 inches from wall, drive another 12 inches
#     drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
#     # Turn 90 degrees to the right
#     drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
#     # Try to line up with the match loader, and spin intake
#     intakeMotors.spin(FORWARD)
#     drivetrain.drive_for(FORWARD, 52, INCHES, wait=True)
#     wait(1, SECONDS)
#     intakeMotors.stop()
#     # Reverse, turn to the right, move under the tube, get the 4 under there
#     drivetrain.drive_for(REVERSE, 6, INCHES, wait=True)
#     drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
#     drivetrain.drive_for(FORWARD, 36, INCHES, wait=True)
#     drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
#     drivetrain.drive_for(FORWARD, 18, INCHES, wait=True)
#     drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
#     intakeMotors.spin(FORWARD)
#     drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
#     wait(1, SECONDS)
#     intakeMotors.stop()
#     # Try to get to the bottom tube to unload
#     drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
#     drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
#     drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
#     drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
#     drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
#     drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
#     drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
    # heightMech.open()
    # drivetrain.drive
    # # Pick up the 6 game elements in the middle bottom, and deposite it in the lower tube
    # drivetrain.drive_for(REVERSE, 12, INCHES, wait=True)
    # drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 36, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
    # intakeMotors.spin(FORWARD)
    # drivetrain.drive_for(FORWARD, 36, INCHES, wait=True)
    # wait(1, SECONDS)
    # intakeMotors.stop()
    # drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 135, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 20, INCHES, wait=True)
    # outtakeFrontMotor.spin(REVERSE)
    # outtakeRearMotor.spin(REVERSE)
    # intakeMotors.spin(REVERSE)
    # wait(5, SECONDS)
    # outtakeFrontMotor.stop()
    # outtakeRearMotor.stop()
    # intakeMotors.stop()
    # # Reverse, go up, try to get the 4 under the tube, score those, then get the 6 in the middle, and score those in the middle tube
    # drivetrain.drive_for(REVERSE, 18, INCHES, wait=True)
    # drivetrain.turn_for(LEFT, 45, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 48, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
    # drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
    # intakeMotors.spin(FORWARD)
    # drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
    # wait(1, SECONDS)
    # intakeMotors.stop() 
    # drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
    # heightMech.open()
    # drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
    # outtakeFrontMotor.spin(FORWARD)
    # outtakeRearMotor.spin(FORWARD)
    # wait(5, SECONDS)
    # outtakeFrontMotor.stop()
    # outtakeRearMotor.stop()
    # drivetrain.drive_for(REVERSE, 12, INCHES, wait=True)
    # heightMech.close()
    # drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 12, INCHES, wait=True)
    # drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 36, INCHES, wait=True)
    # drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
    # intakeMotors.spin(FORWARD)
    # drivetrain.drive_for(FORWARD, 36, INCHES, wait=True)
    # wait(1, SECONDS)
    # intakeMotors.stop()
    # drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 135, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 20, INCHES, wait=True)
    # outtakeFrontMotor.spin(FORWARD)
    # outtakeRearMotor.spin(FORWARD)
    # wait(5, SECONDS)
    # outtakeFrontMotor.stop()
    # outtakeRearMotor.stop()
    # # Go to parking
    # drivetrain.drive_for(REVERSE, 20, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 135, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
    # drivetrain.turn_for(LEFT, 90, DEGREES, wait=True)
    # drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
    # drivetrain.turn_for(RIGHT, 90, DEGREES, wait=True)
    # intakeMotors.spin(FORWARD)
    # drivetrain.drive_for(FORWARD, 24, INCHES, wait=True)
    # wait(2, SECONDS)
    # intakeMotors.stop()
    # drivetrain.stop()

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    drivetrain.drive(FORWARD, 50, PERCENT)
    wait(2, SECONDS)
    drivetrain.stop()
    

# def autonomous_match():
#     brain.screen.clear_screen()
#     brain.screen.print("Autonomous match")


#     #Gunning for the FFA
#     drivetrain.turn_for(LEFT, 45, DEGREES)
#     drivetrain.drive_for(FORWARD, 53.62, INCHES)
#     intakeMotors.spin(FORWARD)
#     drivetrain.turn_for(LEFT, 45, DEGREES)
#     drivetrain.drive_for(FORWARD, 36, INCHES)
#     drivetrain.turn_for(LEFT, 90, DEGREES)
#     wait(5, SECONDS)
#     intakeMotors.stop()
#     drivetrain.drive_for(FORWARD, 24, INCHES)
#     intakeMotors.spin(REVERSE)
#     wait(3, SECONDS)
#     intakeMotors.stop()


#     #Moving to the nearest match loader
#     drivetrain.turn_for(RIGHT, 45, DEGREES)
#     drivetrain.drive_for(FORWARD, 31.69, INCHES)
#     drivetrain.turn_for(RIGHT, 45, DEGREES)
#     intakeMotors.spin(FORWARD)
#     drivetrain.drive_for(FORWARD, 11, INCHES)
#     wait(5, SECONDS)
#     intakeMotors.stop()

#     #Moving to the left long goal
#     drivetrain.drive_for(REVERSE, 3, INCHES)
#     drivetrain.turn_for(RIGHT, 180, DEGREES)
#     drivetrain.drive_for(FORWARD, 43.55, INCHES)
#     heightMech.open()
#     wait(1, SECONDS)
#     outtakeFrontMotor.spin(FORWARD)
#     intakeMotors.spin(FORWARD)
#     wait(2, SECONDS)
#     intakeMotors.stop()
#     heightMech.close()

#     #Moving back to the loader
#     drivetrain.drive_for(REVERSE, 12, INCHES)
#     drivetrain.turn_for(RIGHT, 180, DEGREES)
#     intakeMotors.spin(FORWARD)
#     drivetrain.drive_for(FORWARD, 11, INCHES)
#     wait(5, SECONDS)
#     intakeMotors.stop()

#     #Moving to the long goal
#     drivetrain.drive_for(REVERSE, 10, INCHES)
#     drivetrain.turn_for(RIGHT, 180, DEGREES)
#     drivetrain.drive_for(FORWARD, 10, INCHES)
#     heightMech.open()
#     wait(1, SECONDS)
#     intakeMotors.spin(FORWARD)
#     wait(4, SECONDS)
#     intakeMotors.stop()


    

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    controller = Controller()
    # place driver control in this while loop
    while True:
        turn = 0.85*controller.axis1.position() #0.85 is the for trim value from original code
        drive = 0.85*controller.axis3.position() #0.6 is the lat trim value from original code
        if controller.buttonR1.pressing():
            intakeMotors.spin(FORWARD,100, PERCENT)
        else:
            if controller.buttonR2.pressing():
                intakeMotors.spin(REVERSE,100, PERCENT)
            else:
                intakeMotors.stop()
        if controller.buttonL1.pressing():
            outtakeFrontMotor.spin(FORWARD,100, PERCENT)
        else:
            if controller.buttonL2.pressing():
                outtakeFrontMotor.spin(REVERSE,100, PERCENT)
            else:
                outtakeFrontMotor.stop()
        if (controller.buttonUp.pressing()):
            heightMech.open()
        if (controller.buttonDown.pressing()):
            heightMech.close()
        # controller.buttonLeft.pressed(flap.open)
        # controller.buttonRight.pressed(flap.close)
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.set_pen_width(2)
        brain.screen.print("Drive: " + str(drive))
        left_speed = controller.axis3.position()#drive + turn
        right_speed = controller.axis2.position() #drive - turn
        # Clamp to valid motor range
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        if abs(left_speed) < 3 and abs(right_speed) < 3:
            drivetrain.stop()
        else:
            #leftMotorGroup.set_velocity(left_speed, PERCENT)
            #rightMotorGroup.set_velocity(right_speed, PERCENT)
            #leftMotorGroup.spin(FORWARD)
            #rightMotorGroup.spin(FORWARD)
            drivetrain.drive(FORWARD)

        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()
