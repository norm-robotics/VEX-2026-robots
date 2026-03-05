from config import * # lsp-only
from inverse_kinematics import * # lsp-only
def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    dpadHeading = -1  # -1 = no d-pad heading active
    descoreOpen = False
    matchLoadOpen = False
    # place driver control in this while loop
    while True:
        # --- Heading control ---
        # D-pad snap-to-heading (overrides joystick while held)
        if controller.buttonUp.pressing():
            dpadHeading = 0
        elif controller.buttonDown.pressing():
            dpadHeading = 180
        elif controller.buttonLeft.pressing():
            dpadHeading = 270
        elif controller.buttonRight.pressing():
            dpadHeading = 90
        else:
            dpadHeading = -1

        # Right joystick sets target heading direction
        rightX = controller.axis1.position()  # left/right
        rightY = controller.axis2.position()  # forward/back
        rightMag = math.sqrt(rightX ** 2 + rightY ** 2)

        if dpadHeading >= 0:
            # D-pad heading snap takes priority
            currentHeading = imu.heading()
            headingError = normalize_angle_deg(dpadHeading - currentHeading)
            turn = driver_heading_pid.calculate_angle_error(headingError, brain.timer.time())
        elif rightMag > DEADZONE:
            targetHeading = joystick_to_heading(rightX, rightY)
            currentHeading = imu.heading()
            headingError = normalize_angle_deg(targetHeading - currentHeading)
            turn = driver_heading_pid.calculate_angle_error(headingError, brain.timer.time())
        else:
            turn = 0
            driver_heading_pid.reset()

        xPos = controller.axis4.position()
        yPos = controller.axis3.position()

        # --- Intake (bumpers) ---
        if controller.buttonR2.pressing() and controller.buttonR1.pressing():
            # R1+R2 combo: toggle match load
            if not matchLoadOpen:
                matchLoad.open()
                matchLoadOpen = True
            else:
                matchLoad.close()
                matchLoadOpen = False
            # debounce – wait for release
            while controller.buttonR2.pressing() and controller.buttonR1.pressing():
                wait(10, MSEC)
        elif controller.buttonR2.pressing():
            intakeMG.spin(FORWARD)
        elif controller.buttonR1.pressing():
            intakeMG.spin(REVERSE)
        else:
            intakeMG.stop()

        if controller.buttonL2.pressing():
            outFlex.spin(FORWARD)
        elif controller.buttonL1.pressing():
            outFlex.spin(REVERSE)
        else:
            outFlex.stop()

        # --- Face buttons ---
        # Y / X = heightMech
        if controller.buttonY.pressing():
            heightMech.open()
        elif controller.buttonX.pressing():
            heightMech.close()

        # B = descore toggle
        if controller.buttonB.pressing():
            if not descoreOpen:
                descore.open()
                descoreOpen = True
            else:
                descore.close()
                descoreOpen = False
            # debounce – wait for release
            while controller.buttonB.pressing():
                wait(10, MSEC)

        # A = reset IMU heading
        if controller.buttonA.pressing():
            imu.set_heading(0)
            driver_heading_pid.reset()
            # debounce – wait for release
            while controller.buttonA.pressing():
                wait(10, MSEC)

        XJoystickDrive(xPos, yPos, turn)
        wait(20, MSEC)