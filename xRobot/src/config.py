# Library imports
from vex import *


# Brain and controller should be defined by default
brain=Brain()
controller = Controller()

#================================
#DEFINING MOTORS
#================================
#Bottom DriveTrain Motors
lBottFront = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
lBottBack = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)
rBottFront = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
rBottBack = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)

#Upper DriveTrain Motors
lUpFront = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
lUpBack = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
rUpFront = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
rUpBack = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)

#Intake Motors
intake = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)

#Outtake Motors
outFlex = Motor(Ports.PORT21, GearSetting.RATIO_18_1, False)
outFlap = Motor(Ports.PORT19, GearSetting.RATIO_6_1, True)

#sensor Motors
gps = Gps(Ports.PORT20)
imu = Inertial(Ports.PORT3)

#Pneumatics
matchLoad = Pneumatics(brain.three_wire_port.a)
heightMech = Pneumatics(brain.three_wire_port.b)
descore = Pneumatics(brain.three_wire_port.c)

#MotorGroups (MG)
rearLeftMG = MotorGroup(lBottBack, lUpBack)
rearRightMG = MotorGroup(rBottBack, rUpBack)
frontLeftMG = MotorGroup(lBottFront, lUpFront)
frontRightMG = MotorGroup(rBottFront, rUpFront)
intakeMG = MotorGroup(intake, outFlap)
intakeMG.set_velocity(100, PERCENT)


#Odometry / sensor fusion
WHEEL_DIAMETER = 3.25  # inches – omni wheel diameter for encoder odometry

#Controller Deadzone
DEADZONE = 10
POSITION_TOLERANCE = 2.0   # inches – how close is "arrived"
HEADING_TOLERANCE  = 5.0   # degrees
SETTLE_CYCLES      = 3     # must stay within tolerance this many loops (~60 ms)
TIMEOUT_MS         = 8000  # give up after this long

FIELD_ORIENTED = True

GPS_QUALITY_MIN = 80    
ALPHA_MAX       = 0.9 
SLIP_ACCEL_THRESHOLD = 30.0  # in/s² – encoder vs IMU accel mismatch before scaling down encoder trust
COLLISION_COOLDOWN_MS = 200  # ms – ignore encoders & IMU heading changes after a collision
