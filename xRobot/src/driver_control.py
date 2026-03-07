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
        # Update sensor fusion each tick
        fusion.update()

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
        rightX = (controller.axis1.position() / 100) ** 3 * 100  # cubic curve
        rightY = (controller.axis2.position() / 100) ** 3 * 100  # cubic curve
        rightMag = math.sqrt(rightX**2 + rightY**2)

        if dpadHeading >= 0:
            # D-pad heading snap takes priority
            currentHeading = fusion.get_heading()
            headingError = normalize_angle_deg(dpadHeading - currentHeading)
            turn = chassis_heading_pid.calculate_angle_error(headingError, brain.timer.time())
        elif rightMag > DEADZONE:
            targetHeading = joystick_to_heading(rightX, rightY)
            currentHeading = fusion.get_heading()
            headingError = normalize_angle_deg(targetHeading - currentHeading)
            turn = chassis_heading_pid.calculate_angle_error(headingError, brain.timer.time())
        else:
            turn = 0
            chassis_heading_pid.reset()

        xPos = (controller.axis4.position() / 100) ** 3 * 100  # cubic curve
        yPos = (controller.axis3.position() / 100) ** 3 * 100  # cubic curve

        # --- Intake (bumpers) ---
        
        if controller.buttonR1.pressing():
            intakeMG.spin(FORWARD)
        
        if controller.buttonR2.pressing():
            outFlex.spin(FORWARD, 100, PERCENT)
        

        if controller.buttonL1.pressing():
            intakeMG.spin(REVERSE)
        
        if controller.buttonL2.pressing():
            outFlex.spin(REVERSE, 100, PERCENT)
        
        if not controller.buttonR1.pressing() and not controller.buttonL1.pressing():
            intakeMG.stop()
        if not controller.buttonR2.pressing() and not controller.buttonL2.pressing():
            outFlex.stop()

        # --- Face buttons ---
        # X = heightMech
        if controller.buttonX.pressing():
            if not heightMechOpen:
                heightMech.open()
                heightMechOpen = True
            else:
                heightMech.close()
                heightMechOpen = False
            # debounce – wait for release
            while controller.buttonX.pressing():
                wait(10, MSEC)

        # B = descore toggle
        if controller.buttonY.pressing():
            if not descoreOpen:
                descore.open()
                descoreOpen = True
            else:
                descore.close()
                descoreOpen = False
            # debounce – wait for release
            while controller.buttonY.pressing():
                wait(10, MSEC)

        # A = reset IMU heading
        if controller.buttonA.pressing():
            fusion.set_heading(0)
            chassis_heading_pid.reset()
            # debounce – wait for release
            while controller.buttonA.pressing():
                wait(10, MSEC)

        # B = toggle field oriented drive
        if controller.buttonB.pressing():
            fieldOriented = not fieldOriented
            # debounce – wait for release
            while controller.buttonB.pressing():
                wait(10, MSEC)

        XJoystickDrive(xPos, yPos, turn)
        wait(20, MSEC)