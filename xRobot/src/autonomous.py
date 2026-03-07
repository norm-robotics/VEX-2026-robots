from config import * # lsp-only
from inverse_kinematics import * # lsp-only
def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")

    # Read starting position from GPS
    startX = gps.x_position()
    startY = gps.y_position()

    # Track absolute target position
    posX = startX
    posY = startY
    heading = 0

    # 1. Drive 36 inches to the right
    posX += 36
    while not driveToPoint(posX, posY, heading, drive_state):
        wait(20, MSEC)

    # 2. Rotate 180 degrees
    heading = 180
    while not driveToPoint(posX, posY, heading, drive_state):
        wait(20, MSEC)

    # 3. Drop matchloader
    matchLoad.open()

    # 4. Turn on intake
    intakeMG.spin(FORWARD)

    # 5. Drive down 12 inches
    posY -= 12
    while not driveToPoint(posX, posY, heading, drive_state):
        wait(20, MSEC)

    # 6. Wait 1 second
    wait(1000, MSEC)

    # 7. Drive up 12 inches
    posY += 12
    while not driveToPoint(posX, posY, heading, drive_state):
        wait(20, MSEC)

    # 8. Lift heightMech
    heightMech.open()

    # 9. Lift matchloader
    matchLoad.close()

    # 10. Spin 180 degrees (back to 0)
    heading = 0
    while not driveToPoint(posX, posY, heading, drive_state):
        wait(20, MSEC)

    # 11. Left 4 inches
    posX -= 4
    while not driveToPoint(posX, posY, heading, drive_state):
        wait(20, MSEC)

    # 12. Drive up 12 inches
    posY += 12
    while not driveToPoint(posX, posY, heading, drive_state):
        wait(20, MSEC)

    # 13. Run both intake forward and outtake flex forward
    intakeMG.spin(FORWARD)
    outFlex.spin(FORWARD)