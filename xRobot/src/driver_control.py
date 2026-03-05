from config import * # lsp-only
def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    # place driver control in this while loop
    while True:
        turn = controller.axis1.position()  
        xPos = controller.axis4.position()  
        yPos = controller.axis3.position()
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