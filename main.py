# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       jordanawilkes                                                #
# 	Created:      11/5/2025, 9:50:54 PM                                        #
# 	Description:  V5 project with multi-config autonomous (4 positions)       #
#                                                                              #
# ---------------------------------------------------------------------------- #

""" 400 degrees = 20 inches"""

# Library imports
from vex import *

# === CONFIGURATION SELECTION ===
TEAM_COLOR = "BLUE"    # Options: "BLUE" or "RED"
STARTING_SIDE = "LEFT" # Options: "LEFT" or "RIGHT"

# === CONSTANTS ===
DEADBAND = 5
INTAKE_SPEED = 100
DRIVE_UPDATE_RATE = 20
INCHES_TO_DEGREES_SLOPE = 0.0514113
INCHES_TO_DEGREES_OFFSET = 0.806452

# Brake Settings
DRIVE_BRAKE_STRENGTH_STRONG = 0.7
DRIVE_BRAKE_STRENGTH_WEAK = 0.4
TURN_BRAKE_MULTIPLIER = 1.3
ACTIVE_DRIVE_BRAKE = DRIVE_BRAKE_STRENGTH_STRONG

# PD Control Constants for Drive
KP_DRIVE = 0.25
KD_DRIVE = 0.25
MIN_POWER = 12
MAX_POWER = 80

# PD Control Constants for Inertial Turns
KP_INERTIAL = 0.7
KD_INERTIAL = 0.25
MIN_TURN_POWER = 10
MAX_TURN_POWER = 45

# === DEVICE CONFIGURATION ===
brain = Brain()
controller = Controller(PRIMARY)

# Inertial Sensor
inertial = Inertial(Ports.PORT10)

# Pneumatic Pistons
mg_piston = DigitalOut(brain.three_wire_port.g)
ds_piston = DigitalOut(brain.three_wire_port.f)
ml_piston = DigitalOut(brain.three_wire_port.h)

# Piston toggle states
mg_piston_state = False
ds_piston_state = False
ml_piston_state = False

# Button press tracking
button_a_was_pressed = False
button_x_was_pressed = False
button_b_was_pressed = False

# Drivetrain motors
left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_motor_c = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_a = Motor(Ports.PORT15, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)
right_motor_c = Motor(Ports.PORT16, GearSetting.RATIO_18_1, True)

left_drive = MotorGroup(left_motor_a, left_motor_b, left_motor_c)
right_drive = MotorGroup(right_motor_a, right_motor_b, right_motor_c)

# Configure brake modes
left_drive.set_stopping(COAST)
right_drive.set_stopping(COAST)

# Intake motors
intake_blue = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
intake_small = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)

# === CUSTOM BRAKE VARIABLES ===
previous_left_speed = 0
previous_right_speed = 0

# === CONFIGURATION HELPER ===
def get_turn_multiplier():
    """
    Calculate turn direction multiplier based on side.
    LEFT side: normal (1)
    RIGHT side: mirror (-1)
    """
    return 1 if STARTING_SIDE == "LEFT" else -1

# === INERTIAL SENSOR INITIALIZATION ===
def calibrate_inertial():
    """Calibrate the inertial sensor. Must be done while robot is stationary!"""
    brain.screen.clear_screen()
    brain.screen.print("Config: " + TEAM_COLOR + " - " + STARTING_SIDE)
    brain.screen.new_line()
    brain.screen.print("Calibrating Inertial...")
    brain.screen.new_line()
    brain.screen.print("DO NOT MOVE ROBOT!")
    
    inertial.calibrate()
    
    while inertial.is_calibrating():
        wait(50, MSEC)
    
    brain.screen.new_line()
    brain.screen.print("Calibration Complete!")
    wait(0.1, SECONDS)

# === HELPER FUNCTIONS ===
def apply_deadband(value, deadband=DEADBAND):
    """Apply deadband to joystick input to reduce drift."""
    return 0 if abs(value) < deadband else value

def apply_cubic_scaling(value):
    """Apply cubic scaling to joystick input for finer control."""
    normalized = value / 100.0
    scaled = normalized ** 3
    return scaled * 100.0

def apply_custom_brake(current_speed, previous_speed, brake_strength, is_turning=False):
    """Apply custom brake between BRAKE and COAST modes."""
    if abs(current_speed) > DEADBAND:
        return current_speed
    
    effective_brake = brake_strength
    if is_turning:
        effective_brake = min(1.0, brake_strength * TURN_BRAKE_MULTIPLIER)
    
    brake_factor = 1.0 - effective_brake
    new_speed = previous_speed * brake_factor
    
    if abs(new_speed) < 2:
        return 0
    
    return new_speed

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

def inches_to_degrees(inches):
    """Convert inches to motor encoder degrees using calibration equation."""
    degrees = (inches + INCHES_TO_DEGREES_OFFSET) / INCHES_TO_DEGREES_SLOPE
    return degrees

# === PD CONTROL FUNCTIONS ===
def reset_drive_encoders():
    """Reset all drive motor encoders to zero."""
    left_drive.set_position(0, DEGREES)
    right_drive.set_position(0, DEGREES)

def get_average_encoder_value():
    """Get average encoder value from both sides."""
    return (left_drive.position(DEGREES) + right_drive.position(DEGREES)) / 2.0

def clamp(value, min_val, max_val):
    """Clamp a value between min and max."""
    return max(min_val, min(max_val, value))

def drive_forward_pd(target_degrees, max_speed=MAX_POWER, timeout_sec=5.0):
    """Drive forward using PD control with gyro correction."""
    reset_drive_encoders()
    initial_heading = inertial.heading(DEGREES)
    
    start_time = brain.timer.time(SECONDS)
    previous_error = 0
    
    while True:
        current_position = get_average_encoder_value()
        error = target_degrees - current_position
        
        if abs(error) < 10:
            break
        
        if (brain.timer.time(SECONDS) - start_time) > timeout_sec:
            break
        
        derivative = error - previous_error
        power = (error * KP_DRIVE) + (derivative * KD_DRIVE)
        
        if abs(error) > 50:
            if power > 0:
                power = max(power, MIN_POWER)
            else:
                power = min(power, -MIN_POWER)
        
        power = clamp(power, -max_speed, max_speed)
        
        current_heading = inertial.heading(DEGREES)
        heading_error = initial_heading - current_heading
        
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        
        correction = heading_error * 0.5
        
        left_drive.spin(FORWARD, power + correction, PERCENT)
        right_drive.spin(FORWARD, power - correction, PERCENT)
        
        previous_error = error
        wait(20, MSEC)
    
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)

def turn_to_heading(target_heading, timeout_sec=3.0):
    """Turn to an absolute heading using inertial sensor with PD control."""
    start_time = brain.timer.time(SECONDS)
    previous_error = 0
    
    while True:
        current_heading = inertial.heading(DEGREES)
        error = target_heading - current_heading
        
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        
        if abs(error) < 2:
            break
        
        if (brain.timer.time(SECONDS) - start_time) > timeout_sec:
            break
        
        derivative = error - previous_error
        power = (error * KP_INERTIAL) + (derivative * KD_INERTIAL)
        
        if abs(error) > 10:
            if power > 0:
                power = max(power, MIN_TURN_POWER)
            else:
                power = min(power, -MIN_TURN_POWER)
        
        power = clamp(power, -MAX_TURN_POWER, MAX_TURN_POWER)
        
        left_drive.spin(FORWARD, power, PERCENT)
        right_drive.spin(REVERSE, power, PERCENT)
        
        previous_error = error
        wait(20, MSEC)
    
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)

def turn_relative(degrees, timeout_sec=3.0):
    """Turn relative to current heading using inertial sensor."""
    current_heading = inertial.heading(DEGREES)
    target_heading = (current_heading + degrees) % 360
    turn_to_heading(target_heading, timeout_sec)

# === HIGH-LEVEL AUTONOMOUS FUNCTIONS ===

def moving(inches, speed=MAX_POWER, timeout_sec=5.0):
    """Move forward or backward a specified distance in inches."""
    degrees = inches_to_degrees(inches)
    drive_forward_pd(degrees, speed, timeout_sec)

def turn_right(degrees, timeout_sec=3.0):
    """Turn right - automatically mirrors for RIGHT side."""
    turn_relative(degrees * get_turn_multiplier(), timeout_sec)

def turn_left(degrees, timeout_sec=3.0):
    """Turn left - automatically mirrors for RIGHT side."""
    turn_relative(-degrees * get_turn_multiplier(), timeout_sec)

def turn_to(heading, timeout_sec=3.0):
    """Turn to an absolute heading (0-360 degrees)."""
    turn_to_heading(heading, timeout_sec)

def intake_both(speed=INTAKE_SPEED, duration_sec=None):
    """Run both intake motors at the same speed."""
    intake_blue.spin(FORWARD if speed > 0 else REVERSE, abs(speed), PERCENT)
    intake_small.spin(FORWARD if speed > 0 else REVERSE, abs(speed), PERCENT)
    if duration_sec is not None:
        wait(duration_sec, SECONDS)
        intake_blue.stop()
        intake_small.stop()

def intake_small_motor(speed=INTAKE_SPEED, duration_sec=None):
    """Run the small intake motor."""
    intake_small.spin(FORWARD if speed > 0 else REVERSE, abs(speed), PERCENT)
    if duration_sec is not None:
        wait(duration_sec, SECONDS)
        intake_small.stop()

def intake_blue_motor(speed=INTAKE_SPEED, duration_sec=None):
    """Run the blue intake motor."""
    intake_blue.spin(FORWARD if speed > 0 else REVERSE, abs(speed), PERCENT)
    if duration_sec is not None:
        wait(duration_sec, SECONDS)
        intake_blue.stop()

def stop_intakeB():
    """Stop blue intake motor."""
    intake_blue.stop()

def stop_intakeS():
    """Stop small intake motor."""
    intake_small.stop()

def piston_G(state):
    """Control the G piston (mg_piston on port G)."""
    global mg_piston_state
    if state in ["on", True, 1]:
        mg_piston_state = True
        mg_piston.set(True)
    else:
        mg_piston_state = False
        mg_piston.set(False)

def piston_H(state):
    """Control the H piston (ml_piston on port H)."""
    global ml_piston_state
    if state in ["on", True, 1]:
        ml_piston_state = True
        ml_piston.set(True)
    else:
        ml_piston_state = False
        ml_piston.set(False)

def piston_F(state):
    """Control the F piston (ds_piston on port F)."""
    global ds_piston_state
    if state in ["on", True, 1]:
        ds_piston_state = True
        ds_piston.set(True)
    else:
        ds_piston_state = False
        ds_piston.set(False)

# === AUTONOMOUS ROUTINES ===

def autonomous_blue_left():
    """Original autonomous routine - BLUE team, LEFT side"""
    intake_both(-100, 0.4)
    moving(16.5, 80)
    turn_right(90)
    intake_blue_motor(-100)
    moving(30, 45)
    wait(1, SECONDS)
    stop_intakeB()
    moving(-30, 75)
    turn_left(90)
    moving(25, 75)
    turn_left(90)
    moving(-22)
    intake_both(-100, 4)
    piston_F(1)
    moving(15, 90)
    turn_left(90)
    moving(11.5, 70)
    turn_right(87)
    moving(-37, 80)
    wait(5, SECONDS)
    intake_blue_motor(-100)

def autonomous_blue_right():
    """Mirrored routine - BLUE team, RIGHT side"""
    intake_both(-100, 0.4)
    moving(16.5, 80)
    turn_left(90)
    intake_blue_motor(-100)
    moving(30, 45)
    wait(1, SECONDS)
    stop_intakeB()
    moving(-30, 75)
    turn_right(90)
    moving(25, 75)
    turn_right(90)
    moving(-22)
    intake_both(-100, 4)
    piston_F(1)
    moving(15, 90)
    turn_right(90)
    moving(11.5, 70)
    turn_left(87)
    moving(-37, 80)
    wait(5, SECONDS)
    intake_blue_motor(-100)

def autonomous_red_left():
    """RED team, LEFT side - same movements as blue_left but for red alliance"""
    intake_both(-100, 0.4)
    moving(16.5, 80)
    turn_right(90)
    intake_blue_motor(-100)
    moving(30, 45)
    wait(1, SECONDS)
    stop_intakeB()
    moving(-30, 75)
    turn_left(90)
    moving(25, 75)
    turn_left(90)
    moving(-22)
    intake_both(-100, 4)
    piston_F(1)
    moving(15, 90)
    turn_left(90)
    moving(11.5, 70)
    turn_right(87)
    moving(-37, 80)
    wait(5, SECONDS)
    intake_blue_motor(-100)

def autonomous_red_right():
    """RED team, RIGHT side - mirrored movements"""
    intake_both(-100, 0.4)
    moving(16.5, 80)
    turn_left(90)
    intake_blue_motor(-100)
    moving(30, 45)
    wait(1, SECONDS)
    stop_intakeB()
    moving(-30, 75)
    turn_right(90)
    moving(25, 75)
    turn_right(90)
    moving(-22)
    intake_both(-100, 4)
    piston_F(1)
    moving(15, 90)
    turn_right(90)
    moving(11.5, 70)
    turn_left(87)
    moving(-37, 80)
    wait(5, SECONDS)
    intake_blue_motor(-100)

# === MAIN AUTONOMOUS SELECTOR ===
def autonomous():
    """Main autonomous function - selects correct routine based on configuration."""
    calibrate_inertial()
    
    brain.screen.clear_screen()
    brain.screen.print("Running: " + TEAM_COLOR + " " + STARTING_SIDE)
    brain.screen.new_line()
    
    # Select appropriate autonomous routine
    if TEAM_COLOR == "BLUE" and STARTING_SIDE == "LEFT":
        autonomous_blue_left()
    elif TEAM_COLOR == "BLUE" and STARTING_SIDE == "RIGHT":
        autonomous_blue_right()
    elif TEAM_COLOR == "RED" and STARTING_SIDE == "LEFT":
        autonomous_red_left()
    elif TEAM_COLOR == "RED" and STARTING_SIDE == "RIGHT":
        autonomous_red_right()
    else:
        brain.screen.print("INVALID CONFIG!")
        brain.screen.new_line()
        brain.screen.print("Check TEAM_COLOR")
        brain.screen.new_line()
        brain.screen.print("and STARTING_SIDE")
        return
    
    brain.screen.new_line()
    brain.screen.print("Auton Complete!")

# === DRIVER CONTROL ===
def user_control():
    global previous_left_speed, previous_right_speed
    global mg_piston_state, ds_piston_state, ml_piston_state
    global button_a_was_pressed, button_x_was_pressed, button_b_was_pressed
    
    brain.screen.clear_screen()
    brain.screen.print("Driver Control")
    brain.screen.new_line()
    brain.screen.print(TEAM_COLOR + " - " + STARTING_SIDE)
    
    while True:
        # === DRIVE CONTROL ===
        raw_forward = apply_deadband(controller.axis3.position())
        raw_turn = apply_deadband(controller.axis1.position())
        
        forward = apply_cubic_scaling(raw_forward)
        turn = apply_cubic_scaling(raw_turn)
        
        desired_left_speed = forward + turn
        desired_right_speed = forward - turn
        
        is_turning = abs(raw_turn) > DEADBAND
        
        if abs(raw_forward) < DEADBAND and abs(raw_turn) < DEADBAND:
            left_speed = apply_custom_brake(0, previous_left_speed, ACTIVE_DRIVE_BRAKE, is_turning)
            right_speed = apply_custom_brake(0, previous_right_speed, ACTIVE_DRIVE_BRAKE, is_turning)
        else:
            left_speed = desired_left_speed
            right_speed = desired_right_speed
        
        left_drive.spin(FORWARD, left_speed, PERCENT)
        right_drive.spin(FORWARD, right_speed, PERCENT)
        
        previous_left_speed = left_speed
        previous_right_speed = right_speed
        
        # === INTAKE CONTROL ===
        if controller.buttonL2.pressing():
            control_intake(INTAKE_SPEED, INTAKE_SPEED)
        elif controller.buttonL1.pressing():
            control_intake(-INTAKE_SPEED, -INTAKE_SPEED)
        elif controller.buttonR2.pressing():
            control_intake(INTAKE_SPEED, 0)
        elif controller.buttonR1.pressing():
            control_intake(-INTAKE_SPEED, 0)
        else:
            control_intake(0, 0)
        
        # === PISTON CONTROL ===
        if controller.buttonA.pressing() and not button_a_was_pressed:
            mg_piston_state = not mg_piston_state
            mg_piston.set(mg_piston_state)
            button_a_was_pressed = True
        elif not controller.buttonA.pressing():
            button_a_was_pressed = False
        
        if controller.buttonX.pressing() and not button_x_was_pressed:
            ds_piston_state = not ds_piston_state
            ds_piston.set(ds_piston_state)
            button_x_was_pressed = True
        elif not controller.buttonX.pressing():
            button_x_was_pressed = False
        
        if controller.buttonB.pressing() and not button_b_was_pressed:
            ml_piston_state = not ml_piston_state
            ml_piston.set(ml_piston_state)
            button_b_was_pressed = True
        elif not controller.buttonB.pressing():
            button_b_was_pressed = False
        
        wait(DRIVE_UPDATE_RATE, MSEC)

# === MAIN PROGRAM ===
comp = Competition(user_control, autonomous)

brain.screen.clear_screen()
brain.screen.print("Config: " + TEAM_COLOR + " - " + STARTING_SIDE)
brain.screen.new_line()
brain.screen.print("You got this!! -J")

