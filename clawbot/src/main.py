# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       govind                                                       #
# 	Created:      3/24/2026, 7:59:15 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *

# Brain should be defined by default
brain = Brain()

# 
#

# The controller
controller = Controller()

# Drive motors
leftFrontMotor = Motor(Ports.PORT5, GearSetting.RATIO_18_1, False)
leftRearMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
rightFrontMOtor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
rightRearMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)

# Arm and claw motors will have brake mode set to hold
# Claw motor will have max torque limited
clawMotor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
armMotor1 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
armMotor2 = Motor(Ports.PORT6, GearSetting.RATIO_36_1, False)

armMotorGroup = MotorGroup(armMotor1, armMotor2)

flag = Pneumatics(brain.three_wire_port.h)


# Max motor speed (percent) for motors controlled by buttons
MAX_SPEED = 40

#
# All motors are controlled from this function which is run as a separate thread
#
def drive_task():
    vert = 0
    horizontal = 0

    # setup the claw motor
    clawMotor.set_max_torque(25, PERCENT)
    clawMotor.set_stopping(HOLD)

    # setup the arm motor
    armMotorGroup.set_stopping(HOLD)

    # loop forever
    while True:
        # buttons
        # Three values, max, 0 and -max.
        #
        control_l1  = (controller.buttonL1.pressing() - controller.buttonL2.pressing()) * MAX_SPEED
        control_r1  = (controller.buttonR1.pressing() - controller.buttonR2.pressing()) * MAX_SPEED

        # joystick tank control
        vert = controller.axis3.position()
        horizontal = controller.axis4.position()
        turn = controller.axis1.position()

        # threshold the variable channels so the drive does not
        # move if the joystick axis does not return exactly to 0
        deadband = 15
        if abs(vert) < deadband:
            vert = 0
        if abs(horizontal) < deadband:
            horizontal = 0

        # Now send all drive values to motors

        # The drivetrain
        leftFrontMotor.spin(FORWARD, 0.6*(vert + horizontal + turn), PERCENT)
        leftRearMotor.spin(FORWARD, 0.6*(vert - horizontal + turn), PERCENT)
        rightFrontMOtor.spin(FORWARD, 0.6*(vert - horizontal - turn), PERCENT)
        rightRearMotor.spin(FORWARD, 0.6*(vert + horizontal - turn), PERCENT)

        # Claw and Arm motors
        armMotorGroup.spin(FORWARD, control_l1, PERCENT)
        clawMotor.spin(FORWARD, control_r1, PERCENT)
        # No need to run too fast

        if (controller.buttonX.pressing):
            flag.open()
        elif (controller.buttonB.pressing):
            flag.close()


        sleep(10)

# Run the drive code
drive = Thread(drive_task)

# Python now drops into REPL
