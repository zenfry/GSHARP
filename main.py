# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       jordanawilkes                                                #
# 	Created:      11/5/2025, 9:50:54 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #
# Library imports
from vex import *
# === CONSTANTS ===
DEADBAND = 5
INTAKE_SPEED = 100
DRIVE_UPDATE_RATE = 20
# === DEVICE CONFIGURATION ===
brain = Brain()
controller = Controller(PRIMARY)
# Drivetrain motors
left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_motor_c = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_a = Motor(Ports.PORT16, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)
right_motor_c = Motor(Ports.PORT16, GearSetting.RATIO_18_1, False)
left_drive = MotorGroup(left_motor_a, left_motor_b, left_motor_c)
right_drive = MotorGroup(right_motor_a, right_motor_b, right_motor_c)
# Intake motors
intake_blue = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
intake_small = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)
# === HELPER FUNCTIONS ===
def apply_deadband(value, deadband=DEADBAND):
    """Apply deadband to joystick input to reduce drift."""
    return 0 if abs(value) < deadband else value
def control_intake(blue_speed, small_speed):
    """Control both intake motors with specified speeds."""
    if blue_speed == 0:
        intake_blue.stop()
    else:
        intake_blue.spin(FORWARD if blue_speed > 0 else REVERSE, abs(blue_speed), PERCENT)
    
    if small_speed == 0:
        intake_small.stop()
    else:
        intake_small.spin(FORWARD if small_speed > 0 else REVERSE, abs(small_speed), PERCENT)
# === AUTONOMOUS ===
def autonomous():
    """Autonomous mode - runs for 15 seconds at match start."""
    brain.screen.clear_screen()
    brain.screen.print("Autonomous Mode")
    
    # Add your autonomous code here
    # Example:
    # left_drive.spin_for(FORWARD, 500, DEGREES)
    # right_drive.spin_for(FORWARD, 500, DEGREES)
# === DRIVER CONTROL ===
def user_control():
    """Driver control mode - runs after autonomous."""
    brain.screen.clear_screen()
    brain.screen.print("Driver Control")
    
    while True:
        # === DRIVE CONTROL (ARCADE STYLE) ===
        # Left stick (axis3) controls forward/backward
        # Right stick (axis1) controls turning
        forward = apply_deadband(controller.axis3.position())
        turn = apply_deadband(controller.axis2.position())
        
        left_speed = forward - turn
        right_speed = forward + turn
        
        left_drive.spin(FORWARD, left_speed, PERCENT)
        right_drive.spin(FORWARD, right_speed, PERCENT)
        
        # === INTAKE CONTROL ===
        if controller.buttonL2.pressing():
            # Both intakes forward
            control_intake(INTAKE_SPEED, INTAKE_SPEED)
        elif controller.buttonL1.pressing():
            # Both intakes reverse
            control_intake(-INTAKE_SPEED, -INTAKE_SPEED)
        elif controller.buttonR2.pressing():
            # Only blue intake forward
            control_intake(INTAKE_SPEED, 0)
        elif controller.buttonR1.pressing():
            # Only blue intake reverse
            control_intake(-INTAKE_SPEED, 0)
        else:
            # Stop both intakes
            control_intake(0, 0)
        
        wait(DRIVE_UPDATE_RATE, MSEC)
# === MAIN PROGRAM ===
# Create competition instance
comp = Competition(user_control, autonomous)
# Display startup message
brain.screen.clear_screen()
brain.screen.print("Program Started")
brain.screen.new_line()
brain.screen.print("Ready for Competition")
