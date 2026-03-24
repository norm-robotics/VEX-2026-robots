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
import math

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

# #sensor Motors
# gps = Gps(Ports.PORT20)
imu = Inertial(Ports.PORT20)

#Pneumatics
heightMech = Pneumatics(brain.three_wire_port.a)
flap = Pneumatics(brain.three_wire_port.b)

#MotorGroups (MG)
leftMG = MotorGroup(lBottom, lUpBack, lUpFront)
rightMG = MotorGroup(rBottom, rUpBack, rUpFront)
intakeMG = MotorGroup(intake, outFlap)


#Calling required constructors
drivetrain = SmartDrive(leftMG, rightMG, imu)

maxAccn = 18.0 # in/s^2, experimentally determined max acceleration for the drive
maxVelocity = 340/60*(3.25)*math.pi # in/s, theoretical max velocity of the drive (motor free speed * wheel circumference)

#======================================
#Clamp Function
#======================================
def clamp(val, minVal, maxVal):
    return max(min(val, maxVal), minVal)

#======================================
#Accn Clamp
#======================================
def deltaVMax(dt):
    if dt < 0.02:
        dt = 0.02

    return maxAccn*dt/maxVelocity*100.0

#======================================
#Normalizing Angle
#======================================
def normalizeAngle(angle):
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    
    return angle*math.pi/180.0

#======================================
#PID Class
#======================================
class PID:
    def __init__(self, kp, ki, kd, maxOutput = 100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.maxOutput = maxOutput
        self.integral = 0
        self.derivative = 0
        self.prevError = 0
        self.lastTime = 0

    def step(self, error, currTime):
        if self.lastTime == 0:
            dt = 0.002
        else:
            dt = (currTime - self.lastTime) / 1000.0

        #Finding proportional output
        p = self.kp * error

        #Finding integral output
        self.integral += error * dt
        i = self.ki * self.integral
        if i > self.maxOutput or i < -self.maxOutput:
            i = self.maxOutput
            self.integral = clamp(self.integral, -self.maxOutput, self.maxOutput)

        #Finding derivative output
        if dt > 0:
            self.derivative = (error - self.prevError) / dt
        else:
            self.derivative = 0
        
        d = self.kd * self.derivative

        #Calculating, clamping and returning output
        output = clamp((p + i + d), -self.maxOutput, self.maxOutput)
        return output

    def calcError(self, targetVal, currVal, angle):
        currTime = brain.timer.time()
        if angle:
            error = normalizeAngle(targetVal - currVal)
        else:
            error = targetVal - currVal
    
        return self.step(error, currTime)

angleCorrect = PID(kp = 32, ki = 0.0, kd = 0.0)

#======================================
#Joystick Curve
#======================================
def joystickCurve(position, exponent):
    curve = ((abs(position) / 100.0) ** exponent) * 100.0 #100 is maximum joystick input in all directions

    if position < 0:
        return -curve
    else:
        return curve

#======================================
#Autonomous Skills Code
#======================================

# def 

def getDegree(x2, x1, y2, y1):
    x = x2 - x1
    y = y2 - y1

    return math.atan(y/x) * 180/math.pi


def getDist(x1, x2, y1, y2):
    x = x2 - x1
    y = y2 - y1

    return math.sqrt((x**2) + (y**2))    

def autonomous():
    
    imu.calibrate()
    while(imu.is_calibrating()):
        brain.screen.print("IMU is Calibrating...")
        brain.screen.next_row
        wait(25, MSEC)
    brain.screen.clear_screen()
    brain.screen.print("Autonomous Driving :(")

    # #Moving to the middle of the field
    # theta = getDegree(93.78, 78.19, 70.72, 16.88)
    # dist1 = getDist(93.78, 78.19, 70.72, 16.88)

    # intakeMG.spin(FORWARD, 100, PERCENT)
    # wait(25, MSEC)
    # drivetrain.drive_for(FORWARD, 41, INCHES, wait=True)
    # wait(30, MSEC)
    # drivetrain.turn_to_heading(-77, DEGREES, )
    # wait(25, MSEC)

    # #Making in blocks towards the long goal
    # dist2 = getDist(109.92, 93.78, 70.22, 70.22)

    # drivetrain.drive_for(FORWARD, dist2, INCHES)
    # wait(35, MSEC)
    # intakeMG.stop()
    # wait(50, MSEC)

    # #Reversing to central long goal position
    # dist3 = getDist(62.23 - 7.5, 70.22, 70.22, 106.69)

    # drivetrain.turn_to_heading
    # outFlex.spin(FORWARD, 100, PERCENT)
    # wait(5000, MSEC)
    # outFlex.stop()
    # wait(50, MSEC)
    # heightMech.close()

    
    ##THE CODE BELOW IS ONLY UNTIL THE CODE ABOVE WORKS

    drivetrain.drive_for(FORWARD, 30, INCHES, wait=True)
    wait(25, MSEC)
    drivetrain.turn_to_heading(47, DEGREES, wait=True)
    drivetrain.drive_for(FORWARD, 2, INCHES, wait=True)
    wait(25, MSEC)
    outFlap.spin(FORWARD, 100, PERCENT)
    outFlex.spin(FORWARD, 100, PERCENT)
    wait(2000, MSEC)
    wait(25, MSEC)
    drivetrain.stop()


#======================================
#Driver Skills Code
#======================================

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("DRIVER MODE =(")
    
    #Defining max speeds for moving and turning
    lastForwardVelocity = 0.0

    while True:
        timestamp = brain.timer.time()

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

        currTime = brain.timer.time()
        dt = currTime -timestamp
        
        # turn = joystickCurve(controller.axis1.position(), 2.5)
        # drive = joystickCurve(controller.axis3.position(), 2.5)

        turn = controller.axis1.position()
        drive = controller.axis3. position()
        driveVel = clamp(drive, lastForwardVelocity - deltaVMax(dt), lastForwardVelocity + deltaVMax(dt))
        
        leftSpeed = clamp(driveVel + turn, -100, 100)
        rightSpeed = clamp(driveVel - turn, -100, 100)
        
        if controller.buttonA.pressing():
            leftSpeed, rightSpeed = 70, 70
        elif controller.buttonB.pressing():
            leftSpeed = clamp(driveVel + turn, -100, 100)
            rightSpeed = clamp(driveVel - turn, -100, 100)

        lastForwardVelocity = driveVel
            
        # if (abs(leftSpeed) < 2) or (abs(rightSpeed) < 2):
        #     drivetrain.stop()
        # else:
        #     leftMG.spin(FORWARD, leftSpeed, PERCENT)
        #     rightMG.spin(FORWARD, rightSpeed, PERCENT)

        leftMG.spin(FORWARD, leftSpeed, PERCENT)
        rightMG.spin(FORWARD, rightSpeed, PERCENT)
        
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()    