# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       jordanawilkes                                                #
# 	Created:      11/5/2025, 9:50:54 PM                                        #
# 	Description:  V5 project with PD control, custom brake, and inertial      #
#                                                                              #
# ---------------------------------------------------------------------------- #
# Library imports
from vex import *

# === CONSTANTS ===
DEADBAND = 5
INTAKE_SPEED = 100
DRIVE_UPDATE_RATE = 20

# Brake Settings
DRIVE_BRAKE_STRENGTH_STRONG = 0.7
DRIVE_BRAKE_STRENGTH_WEAK = 0.4
TURN_BRAKE_MULTIPLIER = 1.3
ACTIVE_DRIVE_BRAKE = DRIVE_BRAKE_STRENGTH_STRONG

# PD Control Constants for Drive
KP_DRIVE = 0.25  # Proportional gain for driving straight
KD_DRIVE = 0.25  # Derivative gain for driving straight
MIN_POWER = 12   # Minimum power to overcome friction
MAX_POWER = 80   # Maximum power for safety

# PD Control Constants for Inertial Turns
KP_INERTIAL = 0.7   # Proportional gain for inertial turning
KD_INERTIAL = 0.2  # Derivative gain for inertial turning
MIN_TURN_POWER = 10
MAX_TURN_POWER = 45

# === DEVICE CONFIGURATION ===
brain = Brain()
controller = Controller(PRIMARY)

# Inertial Sensor (change port as needed)
inertial = Inertial(Ports.PORT10)

# Pneumatic Pistons (DigitalOut for single solenoids)
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

# === INERTIAL SENSOR INITIALIZATION ===
def calibrate_inertial():
    """Calibrate the inertial sensor. Must be done while robot is stationary!"""
    brain.screen.clear_screen()
    brain.screen.print("Calibrating Inertial...")
    brain.screen.new_line()
    brain.screen.print("DO NOT MOVE ROBOT!")
    
    inertial.calibrate()
    
    while inertial.is_calibrating():
        wait(50, MSEC)
    
    brain.screen.new_line()
    brain.screen.print("Calibration Complete!")
    wait(0.5, SECONDS)

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

def drive_forward_pd(target_degrees, timeout_sec=5.0):
    """
    Drive forward using PD (Proportional-Derivative) control with gyro correction.
    
    Args:
        target_degrees: Distance to travel in encoder degrees
        timeout_sec: Maximum time to wait (safety)
    """
    reset_drive_encoders()
    initial_heading = inertial.heading(DEGREES)
    
    brain.screen.print("PD Drive: " + str(target_degrees) + " deg")
    brain.screen.new_line()
    
    start_time = brain.timer.time(SECONDS)
    previous_error = 0
    
    while True:
        # Calculate error
        current_position = get_average_encoder_value()
        error = target_degrees - current_position
        
        # Check if we're done
        if abs(error) < 10:  # Within 10 degrees = close enough
            break
        
        # Timeout check
        if (brain.timer.time(SECONDS) - start_time) > timeout_sec:
            brain.screen.print("Timeout!")
            brain.screen.new_line()
            break
        
        # Calculate derivative (rate of change of error)
        derivative = error - previous_error
        
        # Calculate PD power
        power = (error * KP_DRIVE) + (derivative * KD_DRIVE)
        
        # Add minimum power to overcome friction (only when far from target)
        if abs(error) > 50:
            if power > 0:
                power = max(power, MIN_POWER)
            else:
                power = min(power, -MIN_POWER)
        
        # Clamp to safe limits
        power = clamp(power, -MAX_POWER, MAX_POWER)
        
        # Gyro correction to drive straight
        current_heading = inertial.heading(DEGREES)
        heading_error = initial_heading - current_heading
        
        # Normalize heading error to -180 to 180
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        
        # Apply correction (small proportional adjustment)
        correction = heading_error * 0.5
        
        # Apply power with correction
        left_drive.spin(FORWARD, power + correction, PERCENT)
        right_drive.spin(FORWARD, power - correction, PERCENT)
        
        # Store error for next iteration
        previous_error = error
        
        wait(20, MSEC)
    
    # Stop with brake
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    
    brain.screen.print("Final: " + str(get_average_encoder_value()))
    brain.screen.new_line()

def turn_to_heading(target_heading, timeout_sec=3.0):
    """
    Turn to an absolute heading using inertial sensor with PD control.
    
    Args:
        target_heading: Target heading in degrees (0-360)
        timeout_sec: Maximum time to wait
    """
    brain.screen.print("Turn to: " + str(target_heading) + " deg")
    brain.screen.new_line()
    
    start_time = brain.timer.time(SECONDS)
    previous_error = 0
    
    while True:
        current_heading = inertial.heading(DEGREES)
        
        # Calculate error (shortest path)
        error = target_heading - current_heading
        
        # Normalize error to -180 to 180 (shortest turn direction)
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        
        # Check if done
        if abs(error) < 2:  # Within 2 degrees
            break
        
        # Timeout check
        if (brain.timer.time(SECONDS) - start_time) > timeout_sec:
            brain.screen.print("Turn timeout!")
            brain.screen.new_line()
            break
        
        # Calculate derivative
        derivative = error - previous_error
        
        # Calculate PD power
        power = (error * KP_INERTIAL) + (derivative * KD_INERTIAL)
        
        # Add minimum power
        if abs(error) > 10:
            if power > 0:
                power = max(power, MIN_TURN_POWER)
            else:
                power = min(power, -MIN_TURN_POWER)
        
        # Clamp power
        power = clamp(power, -MAX_TURN_POWER, MAX_TURN_POWER)
        
        # Apply power (positive error = turn right)
        left_drive.spin(FORWARD, power, PERCENT)
        right_drive.spin(REVERSE, power, PERCENT)
        
        # Store error for next iteration
        previous_error = error
        
        wait(20, MSEC)
    
    # Stop with brake
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    
    brain.screen.print("Heading: " + str(inertial.heading(DEGREES)))
    brain.screen.new_line()

def turn_relative(degrees, timeout_sec=3.0):
    """
    Turn relative to current heading using inertial sensor.
    
    Args:
        degrees: Degrees to turn (positive = right, negative = left)
        timeout_sec: Maximum time to wait
    """
    current_heading = inertial.heading(DEGREES)
    target_heading = (current_heading + degrees) % 360
    turn_to_heading(target_heading, timeout_sec)

# === HIGH-LEVEL AUTONOMOUS FUNCTIONS ===

def moving(degrees, timeout_sec=5.0):
    """
    Move forward or backward a specified number of degrees.
    Positive = forward, Negative = backward
    """
    drive_forward_pd(degrees, timeout_sec)

def turn_right(degrees, timeout_sec=3.0):
    """Turn right a specified number of degrees using inertial sensor."""
    turn_relative(degrees, timeout_sec)

def turn_left(degrees, timeout_sec=3.0):
    """Turn left a specified number of degrees using inertial sensor."""
    turn_relative(-degrees, timeout_sec)

def turn_to(heading, timeout_sec=3.0):
    """Turn to an absolute heading (0-360 degrees)."""
    turn_to_heading(heading, timeout_sec)

def intake_both(speed=INTAKE_SPEED, duration_sec=None):
    """
    Run both intake motors at the same speed.
    
    Args:
        speed: Motor speed (-100 to 100, negative = reverse)
        duration_sec: If specified, run for this duration then stop
    """
    intake_blue.spin(FORWARD if speed > 0 else REVERSE, abs(speed), PERCENT)
    intake_small.spin(FORWARD if speed > 0 else REVERSE, abs(speed), PERCENT)
    if duration_sec is not None:
        wait(duration_sec, SECONDS)
        intake_blue.stop()
        intake_small.stop()

def intake_small_motor(speed=INTAKE_SPEED, duration_sec=None):
    """
    Run the small intake motor.
    
    Args:
        speed: Motor speed (-100 to 100, negative = reverse)
        duration_sec: If specified, run for this duration then stop
    """
    intake_small.spin(FORWARD if speed > 0 else REVERSE, abs(speed), PERCENT)
    if duration_sec is not None:
        wait(duration_sec, SECONDS)
        intake_small.stop()

def intake_blue_motor(speed=INTAKE_SPEED, duration_sec=None):
    """
    Run the blue intake motor.
    
    Args:
        speed: Motor speed (-100 to 100, negative = reverse)
        duration_sec: If specified, run for this duration then stop
    """
    intake_blue.spin(FORWARD if speed > 0 else REVERSE, abs(speed), PERCENT)
    if duration_sec is not None:
        wait(duration_sec, SECONDS)
        intake_blue.stop()

def stop_intakes():
    """Stop all intake motors."""
    intake_blue.stop()
    intake_small.stop()

def piston_G(state):
    """
    Control the G piston (mg_piston on port G).
    
    Args:
        state: "on" or "off" (can also use True/False or 1/0)
    """
    global mg_piston_state
    if state in ["on", True, 1]:
        mg_piston_state = True
        mg_piston.set(True)
    else:
        mg_piston_state = False
        mg_piston.set(False)

def piston_H(state):
    """
    Control the H piston (ml_piston on port H).
    
    Args:
        state: "on" or "off" (can also use True/False or 1/0)
    """
    global ml_piston_state
    if state in ["on", True, 1]:
        ml_piston_state = True
        ml_piston.set(True)
    else:
        ml_piston_state = False
        ml_piston.set(False)

def piston_F(state):
    """
    Control the F piston (ds_piston on port F).
    
    Args:
        state: "on" or "off" (can also use True/False or 1/0)
    """
    global ds_piston_state
    if state in ["on", True, 1]:
        ds_piston_state = True
        ds_piston.set(True)
    else:
        ds_piston_state = False
        ds_piston.set(False)

# === AUTONOMOUS ===
def autonomous():
    """Autonomous mode - build your routine here!"""
    brain.screen.clear_screen()
    brain.screen.print("Autonomous Started")
    brain.screen.new_line()
    
    # Calibrate inertial at start
    calibrate_inertial()
    
    brain.screen.print("Test 1: 90 deg turns")
    brain.screen.new_line()
    turn_right(180)
    wait(1, SECONDS)
    
    brain.screen.new_line()
    brain.screen.print("Auton Complete!")
    
# === INERTIAL TUNING TEST FUNCTION ===
def test_inertial_tuning():
    """Use this function to test and tune your inertial PD values."""
    brain.screen.clear_screen()
    brain.screen.print("Inertial Tuning Tests")
    brain.screen.new_line()
    
    # Calibrate first
    calibrate_inertial()
    
    brain.screen.print("KP_I:" + str(KP_INERTIAL) + " KD_I:" + str(KD_INERTIAL))
    brain.screen.new_line()
    
    wait(1, SECONDS)
    
    # Test 1: 90 degree turns
    brain.screen.print("Test 1: 90 deg turns")
    brain.screen.new_line()
    turn_right(90)
    wait(1, SECONDS)
    turn_right(90)
    wait(1, SECONDS)
    turn_right(90)
    wait(1, SECONDS)
    turn_right(90)
    wait(1, SECONDS)
    
    # Test 2: Turn to absolute headings
    brain.screen.print("Test 2: Absolute headings")
    brain.screen.new_line()
    turn_to(0)
    wait(0.5, SECONDS)
    turn_to(45)
    wait(0.5, SECONDS)
    turn_to(180)
    wait(0.5, SECONDS)
    turn_to(270)
    wait(0.5, SECONDS)
    turn_to(0)
    wait(1, SECONDS)
    
    # Test 3: Drive straight with gyro
    brain.screen.print("Test 3: Straight drive")
    brain.screen.new_line()
    moving(1000)
    wait(0.5, SECONDS)
    moving(-1000)
    wait(1, SECONDS)
    
    # Test 4: Square pattern
    brain.screen.print("Test 4: Square")
    brain.screen.new_line()
    for i in range(4):
        moving(600)
        wait(0.3, SECONDS)
        turn_right(90)
        wait(0.3, SECONDS)
    
    brain.screen.new_line()
    brain.screen.print("Tuning Tests Complete!")
    brain.screen.print("Final heading: " + str(inertial.heading(DEGREES)))

# === DRIVER CONTROL ===
def user_control():
    """Driver control mode - runs after autonomous."""
    global previous_left_speed, previous_right_speed
    global mg_piston_state, ds_piston_state, ml_piston_state
    global button_a_was_pressed, button_x_was_pressed, button_b_was_pressed
    
    brain.screen.clear_screen()
    brain.screen.print("Driver Control")
    brain.screen.new_line()
    brain.screen.print("Brake: " + str(ACTIVE_DRIVE_BRAKE))
    brain.screen.new_line()
    brain.screen.print("Heading: " + str(inertial.heading(DEGREES)))
    
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
brain.screen.print("Program Started")
brain.screen.new_line()
brain.screen.print("Inertial Sensor Ready")
brain.screen.new_line()
brain.screen.print("Place on flat surface")
brain.screen.new_line()
brain.screen.print("for calibration")
