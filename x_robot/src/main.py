# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       anthony                                                      #
# 	Created:      2/21/2026, 2:28:33 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
import math

# Defines
brain = Brain()
gps = Gps(Ports.PORT12)
frontRightUpperMotor = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
frontRightLowerMotor = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
frontRightMotors = MotorGroup(frontRightUpperMotor,frontRightLowerMotor)
frontLeftUpperMotor = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
frontLeftLowerMotor = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)
frontLeftMotors = MotorGroup(frontLeftUpperMotor,frontLeftLowerMotor)
rearRightUpperMotor = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
rearRightLowerMotor = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
rearRightMotors = MotorGroup(rearRightUpperMotor,rearRightLowerMotor)
rearLeftUpperMotor = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
rearLeftLowerMotor = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)
rearLeftMotors = MotorGroup(rearLeftUpperMotor,rearLeftLowerMotor)
intakeMotorGround = Motor(Ports.PORTFIXEME, GearSetting.RATIO_6_1)
intakeMotorBig = Motor(Ports.PORTFIXEME, GearSetting.RATIO_18_1)
intakeMotors = MotorGroup(intakeMotorGround,intakeMotorBig)
outtakeMotorGround = Motor(Ports.PORTFIXEME, GearSetting.RATIO_6_1)
outtakeMotorBig = Motor(Ports.PORTFIXEME, GearSetting.RATIO_6_1)
outtakeMotors = MotorGroup(outtakeMotorGround,outtakeMotorBig)
matchLoadMech = Pneumatics(brain.three_wire_port.a)
heightMech = Pneumatics(brain.three_wire_port.b)
flap = Pneumatics(brain.three_wire_port.c)

class PIDController:
    """PID controller for motor velocity control"""
    def __init__(self, kp, ki, kd, max_output=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0
        self.previous_error = 0
        self.last_time = 0
    
    def calculate(self, target_velocity, current_velocity, timestamp):
        """Calculate PID output for motor control"""
        error = target_velocity - current_velocity
        
        # Calculate time delta
        if self.last_time == 0:
            dt = 0.02  # Default 20ms if first call
        else:
            dt = (timestamp - self.last_time) / 1000.0  # Convert to seconds
        
        # Proportional term
        p_output = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_output), -self.max_output)
        i_output = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0
        d_output = self.kd * derivative
        
        # Total output with saturation
        output = p_output + i_output + d_output
        output = max(min(output, self.max_output), -self.max_output)
        
        # Update state
        self.previous_error = error
        self.last_time = timestamp
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.integral = 0
        self.previous_error = 0
        self.last_time = 0


# PID Controllers for each motor group
# Tuning parameters (adjust these based on your motor characteristics)
front_right_pid = PIDController(kp=0.5, ki=0.1, kd=0.1, max_output=100)
front_left_pid = PIDController(kp=0.5, ki=0.1, kd=0.1, max_output=100)
rear_right_pid = PIDController(kp=0.5, ki=0.1, kd=0.1, max_output=100)
rear_left_pid = PIDController(kp=0.5, ki=0.1, kd=0.1, max_output=100)





def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    controller = Controller()

    # place driver control in this while loop
    while True:
        rotation = controller.axis2.position()
        Ypos = -controller.axis3.position()
        Xpos = -controller.axis4.position()
        if controller.buttonR1.pressing():
            intakeMotors.spin(FORWARD)
        else:
            if controller.buttonR2.pressing():
                intakeMotors.spin(REVERSE)
            else:
                intakeMotors.stop()
        if controller.buttonL1.pressing():
            outtakeMotors.spin(FORWARD)
        else:
            if controller.buttonL2.pressing():
                outtakeMotors.spin(REVERSE)
            else:
                outtakeMotors.stop()
        if controller.buttonUp.pressing():
            heightMech.open()
        else:
            heightMech.close()
        if controller.buttonLeft.pressing():
            flap.open()
        else:
            flap.close()
        if controller.buttonRight.pressing():
            matchLoadMech.open()
        else:
            matchLoadMech.close()
        XDriveJoystick(Ypos, Xpos, rotation)
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()

def XDriveJoystick(Xpos, Ypos, turn):
    frontRightMotorMove = Ypos - Xpos - turn
    frontLeftMotorMove = -Ypos - Xpos - turn
    rearRightMotorMove = Ypos + Xpos - turn
    rearLeftMotorMove = -Ypos + Xpos - turn
    



def fieldOrientedControl(Xpos, Ypos, turn):
    heading = gps.heading()
    headingRad = heading * (3.14159 / 180)
    rotatedX = Xpos * math.cos(headingRad) - Ypos * math.sin(headingRad)
    rotatedY = Xpos * math.sin(headingRad) + Ypos * math.cos(headingRad)
    XDriveJoystick(rotatedX, rotatedY, turn)




def pidControl(front_right_target, front_left_target, rear_right_target, rear_left_target):
    """Apply PID control to set motor velocities"""
    timestamp = brain.timer.time(MSEC)
    
    # Get current velocities (in percent)
    fr_current = frontRightMotors.velocity(VelocityUnits.PERCENT)
    fl_current = frontLeftMotors.velocity(VelocityUnits.PERCENT)
    rr_current = rearRightMotors.velocity(VelocityUnits.PERCENT)
    rl_current = rearLeftMotors.velocity(VelocityUnits.PERCENT)
    
    # Calculate PID outputs
    fr_output = front_right_pid.calculate(front_right_target, fr_current, timestamp)
    fl_output = front_left_pid.calculate(front_left_target, fl_current, timestamp)
    rr_output = rear_right_pid.calculate(rear_right_target, rr_current, timestamp)
    rl_output = rear_left_pid.calculate(rear_left_target, rl_current, timestamp)
    
    # Apply outputs to motors
    frontRightMotors.spin(FORWARD, fr_output, VelocityUnits.PERCENT)
    frontLeftMotors.spin(FORWARD, fl_output, VelocityUnits.PERCENT)
    rearRightMotors.spin(FORWARD, rr_output, VelocityUnits.PERCENT)
    rearLeftMotors.spin(FORWARD, rl_output, VelocityUnits.PERCENT)
