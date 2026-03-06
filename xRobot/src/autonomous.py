from config import * # lsp-only
def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    intake.spin(FORWARD)
    matchLoad.open()
    drive_state = DriveState()
    driveToPoint(-48, 48, 90, drive_state)
    driveToPoint(-72, 48, 90, drive_state)
    heightMech.open()
    driveToPoint(-24, 48, 90, drive_state)
    outFlex.spin(FORWARD)

