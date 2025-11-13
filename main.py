# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       jordanawilkes                                                #
# 	Created:      11/5/2025, 9:50:54 PM                                        #
# 	Description:  V5 project with PD control and custom brake                 #
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

# PD Control Constants
KP_DRIVE = 0.25  # Proportional gain for driving straight
KD_DRIVE = 0.25  # Derivative gain for driving straight (tune this!)
KP_TURN = 0.15   # Proportional gain for turning
KD_TURN = 0.05   # Derivative gain for turning (tune this!)
MIN_POWER = 12   # Minimum power to overcome friction
MAX_POWER = 80   # Maximum power for safety

#205=90

# === DEVICE CONFIGURATION ===
brain = Brain()
controller = Controller(PRIMARY)

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
    Drive forward using PD (Proportional-Derivative) control.
    
    Args:
        target_degrees: Distance to travel in encoder degrees
        timeout_sec: Maximum time to wait (safety)
    """
    reset_drive_encoders()
    
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
        
        # Apply power
        left_drive.spin(FORWARD, power, PERCENT)
        right_drive.spin(FORWARD, power, PERCENT)
        
        # Store error for next iteration
        previous_error = error
        
        wait(20, MSEC)
    
    # Stop with brake
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    
    brain.screen.print("Final: " + str(get_average_encoder_value()))
    brain.screen.new_line()

def turn_pd(target_degrees, timeout_sec=3.0):
    """
    Turn using PD (Proportional-Derivative) control.
    
    Args:
        target_degrees: Angle to turn (positive = right, negative = left)
        timeout_sec: Maximum time to wait
    """
    reset_drive_encoders()
    
    brain.screen.print("PD Turn: " + str(target_degrees) + " deg")
    brain.screen.new_line()
    
    start_time = brain.timer.time(SECONDS)
    previous_error = 0
    
    while True:
        # For turning, we use the difference between left and right
        current_position = (left_drive.position(DEGREES) - right_drive.position(DEGREES)) / 2.0
        error = target_degrees - current_position
        
        # Check if done
        if abs(error) < 5:  # Within 5 degrees for turns
            break
        
        # Timeout check
        if (brain.timer.time(SECONDS) - start_time) > timeout_sec:
            brain.screen.print("Turn timeout!")
            brain.screen.new_line()
            break
        
        # Calculate derivative
        derivative = error - previous_error
        
        # Calculate PD power
        power = (error * KP_TURN) + (derivative * KD_TURN)
        
        # Add minimum power
        if abs(error) > 20:
            if power > 0:
                power = max(power, MIN_POWER)
            else:
                power = min(power, -MIN_POWER)
        
        # Clamp power
        power = clamp(power, -MAX_POWER, MAX_POWER)
        
        # Apply opposite power to each side for turning
        left_drive.spin(FORWARD, power, PERCENT)
        right_drive.spin(REVERSE, power, PERCENT)
        
        # Store error for next iteration
        previous_error = error
        
        wait(20, MSEC)
    
    # Stop with brake
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    
    brain.screen.print("Turn complete")
    brain.screen.new_line()

# === HIGH-LEVEL AUTONOMOUS FUNCTIONS ===

def moving(degrees, timeout_sec=5.0):
    """
    Move forward or backward a specified number of degrees.
    Positive = forward, Negative = backward
    """
    drive_forward_pd(degrees, timeout_sec)

def turn_right(degrees, timeout_sec=3.0):
    """Turn right a specified number of degrees."""
    turn_pd(degrees, timeout_sec)

def turn_left(degrees, timeout_sec=3.0):
    """Turn left a specified number of degrees."""
    turn_pd(-degrees, timeout_sec)

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

def intake_small(speed=INTAKE_SPEED, duration_sec=None):
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

def intake_blue(speed=INTAKE_SPEED, duration_sec=None):
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
    
    piston_G("on")
    wait(0.3, SECONDS)
    
    intake_both(100, 1.0)
    
    moving(500)
    wait(0.2, SECONDS)
    
    piston_G("off")
    wait(0.2, SECONDS)
    
    turn_right(200)
    wait(0.2, SECONDS)
    
    moving(300)
    wait(0.2, SECONDS)
    
    intake_both(-100, 0.5)
    stop_intakes()
    
    moving(-300)
    
    brain.screen.new_line()
    brain.screen.print("Auton Complete!")
    
    
# === PD TUNING TEST FUNCTION ===
def test_pd_tuning():
    """Use this function to test and tune your PD values."""
    brain.screen.clear_screen()
    brain.screen.print("PD Tuning Tests")
    brain.screen.new_line()
    brain.screen.print("KP_D:" + str(KP_DRIVE) + " KD_D:" + str(KD_DRIVE))
    brain.screen.new_line()
    brain.screen.print("KP_T:" + str(KP_TURN) + " KD_T:" + str(KD_TURN))
    brain.screen.new_line()
    
    wait(1, SECONDS)
    
    # Test 1: Quick acceleration test
    brain.screen.print("Test 1: Quick Start/Stop")
    brain.screen.new_line()
    moving(500)
    wait(0.5, SECONDS)
    
    # Test 2: Precision positioning
    brain.screen.print("Test 2: Precision Moves")
    brain.screen.new_line()
    moving(200)
    wait(0.3, SECONDS)
    moving(200)
    wait(0.3, SECONDS)
    moving(200)
    wait(0.5, SECONDS)
    
    # Test 3: Sharp turn
    brain.screen.print("Test 3: Sharp Right Turn")
    brain.screen.new_line()
    turn_right(300)
    wait(0.5, SECONDS)
    
    # Test 4: Drive and turn sequence
    brain.screen.print("Test 4: Drive + Turn")
    brain.screen.new_line()
    moving(800)
    wait(0.3, SECONDS)
    turn_left(300)
    wait(0.3, SECONDS)
    moving(400)
    wait(0.5, SECONDS)
    
    # Test 5: Reverse
    brain.screen.print("Test 5: Reverse")
    brain.screen.new_line()
    moving(-600)
    wait(0.5, SECONDS)
    
    # Test 6: Quick direction changes
    brain.screen.print("Test 6: Quick Changes")
    brain.screen.new_line()
    moving(300)
    wait(0.2, SECONDS)
    turn_right(150)
    wait(0.2, SECONDS)
    moving(-300)
    wait(0.2, SECONDS)
    turn_left(150)
    
    brain.screen.new_line()
    brain.screen.print("Tuning Tests Complete!")
    
# To use the tuning tests, temporarily change autonomous() to call test_pd_tuning()

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
brain.screen.print("PD Control Ready")
brain.screen.new_line()
brain.screen.print("KP_D:" + str(KP_DRIVE) + " KD_D:" + str(KD_DRIVE))
brain.screen.new_line()
brain.screen.print("KP_T:" + str(KP_TURN) + " KD_T:" + str(KD_TURN))
