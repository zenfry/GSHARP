# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       jordanawilkes                                                #
# 	Created:      11/5/2025, 9:50:54 PM                                        #
# 	Description:  V5 project with proportional control and custom brake       #
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

# Proportional Control Constants
KP_DRIVE = 0.42  # Proportional gain for driving straight (tune this!)
KP_TURN = 0.2   # Proportional gain for turning (tune this!)
MIN_POWER = 15  # Minimum power to overcome friction
MAX_POWER = 80  # Maximum power for safety

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
left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
left_motor_c = Motor(Ports.PORT13, GearSetting.RATIO_18_1, True)
right_motor_a = Motor(Ports.PORT15, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)
right_motor_c = Motor(Ports.PORT16, GearSetting.RATIO_18_1, False)

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

# === PROPORTIONAL CONTROL FUNCTIONS ===
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

def drive_forward_proportional(target_degrees, timeout_sec=5.0):
    """
    Drive forward using proportional control.
    
    Args:
        target_degrees: Distance to travel in encoder degrees
        timeout_sec: Maximum time to wait (safety)
    """
    reset_drive_encoders()
    
    brain.screen.print("Driving: " + str(target_degrees) + " deg")
    brain.screen.new_line()
    
    start_time = brain.timer.time(SECONDS)
    
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
        
        # Calculate proportional power
        power = error * KP_DRIVE
        
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
        
        wait(20, MSEC)
    
    # Stop with brake
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    
    brain.screen.print("Final: " + str(get_average_encoder_value()))
    brain.screen.new_line()

def turn_proportional(target_degrees, timeout_sec=3.0):
    """
    Turn using proportional control.
    
    Args:
        target_degrees: Angle to turn (positive = right, negative = left)
        timeout_sec: Maximum time to wait
    """
    reset_drive_encoders()
    
    brain.screen.print("Turning: " + str(target_degrees) + " deg")
    brain.screen.new_line()
    
    start_time = brain.timer.time(SECONDS)
    
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
        
        # Calculate proportional power
        power = error * KP_TURN
        
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
        
        wait(20, MSEC)
    
    # Stop with brake
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    
    brain.screen.print("Turn complete")
    brain.screen.new_line()

# === AUTONOMOUS ===
def autonomous():
    """Autonomous mode with proportional control testing."""
    brain.screen.clear_screen()
    brain.screen.print("Autonomous - P Control")
    brain.screen.new_line()
    
    wait(0.5, SECONDS)
    
    # Test 1: Drive forward 1000 degrees (about 2.78 rotations)
    brain.screen.print("Test 1: Forward")
    brain.screen.new_line()
    drive_forward_proportional(1000)
    wait(1, SECONDS)
    
    # Test 2: Turn right 90 degrees (approximate - tune this value!)
    brain.screen.print("Test 2: Turn Right")
    brain.screen.new_line()
    turn_proportional(200)  # This value depends on your robot's width
    wait(1, SECONDS)
    
    # Test 3: Drive forward again
    brain.screen.print("Test 3: Forward")
    brain.screen.new_line()
    drive_forward_proportional(30)
    wait(1, SECONDS)
    
    # Test 4: Turn left
    brain.screen.print("Test 4: Turn Left")
    brain.screen.new_line()
    turn_proportional(-200)
    wait(1, SECONDS)
    
    # Test 5: Drive backward
    brain.screen.print("Test 5: Backward")
    brain.screen.new_line()
    drive_forward_proportional(-600)
    
    brain.screen.new_line()
    brain.screen.print("Auton Complete!")

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
        
        desired_left_speed = forward - turn
        desired_right_speed = forward + turn
        
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
brain.screen.print("P Control Ready")
brain.screen.new_line()
brain.screen.print("KP_DRIVE: " + str(KP_DRIVE))
brain.screen.new_line()
brain.screen.print("KP_TURN: " + str(KP_TURN))
