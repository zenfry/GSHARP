# Library imports
from vex import *
import math

#  CONSTANTS
DEADBAND = 5
INTAKE_SPEED = 100
DRIVE_UPDATE_RATE = 20

# Drivetrain Configuration
GEAR_RATIO = 18  
NUM_WHEELS = 6  
WHEEL_DIAMETER_CM = 82.55  # Wheel diameter in millimeters (3.25" = 82.55cm)

# Brake Settings
DRIVE_BRAKE_STRENGTH_STRONG = 0.7
DRIVE_BRAKE_STRENGTH_WEAK = 0.4
TURN_BRAKE_MULTIPLIER = 1.3
ACTIVE_DRIVE_BRAKE = DRIVE_BRAKE_STRENGTH_STRONG
Tolerance = 15

# PID Control Constants for Drive
KP_DRIVE = 0.25
KI_DRIVE = 0.001  
KD_DRIVE = 0.25
MIN_POWER = 12
MAX_POWER = 80
INTEGRAL_MAX = 3000  

# PID Control Constants for Inertial Turns
KP_INERTIAL = 0.6
KI_INERTIAL = 0.002  
KD_INERTIAL = 0.25
MIN_TURN_POWER = 10
MAX_TURN_POWER = 45
TURN_INTEGRAL_MAX = 1000  

#  DEVICE CONFIGURATION
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

#  CUSTOM BRAKE VARIABLES
previous_left_speed = 0
previous_right_speed = 0

#  INERTIAL SENSOR INITIALIZATION
def calibrate_inertial():
    brain.screen.print("Calibrating Inertial...")
    brain.screen.new_line()
    brain.screen.print("DO NOT MOVE ROBOT!")
   
    inertial.calibrate()
   
    while inertial.is_calibrating():
        wait(50, MSEC)
   

#  DISTANCE CONVERSION FUNCTIONS
def cm_to_degrees(cm):
   
    # Calculate wheel circumference in cm
    circumference_cm = math.pi * WHEEL_DIAMETER_CM
   
    # Calculate wheel rotations needed
    wheel_rotations = cm / circumference_cm
   
    motor_4 = wheel_rotations * 360 * GEAR_RATIO

    motor_degrees = motor_4 / 3.2512
   
    return motor_degrees

#  HELPER FUNCTIONS
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

#  PID CONTROL FUNCTIONS
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

def drive_forward_pid(target_degrees, max_speed=MAX_POWER, timeout_sec=5.0):
    """Drive forward using PID control with gyro correction."""
    reset_drive_encoders()
    initial_heading = inertial.heading(DEGREES)
   
    start_time = brain.timer.time(SECONDS)
    previous_error = 0
    integral = 0


    while True:
        current_position = get_average_encoder_value()
        error = target_degrees - current_position
       
        if abs(error) < 10:
            break
       
        if (brain.timer.time(SECONDS) - start_time) > timeout_sec:
            break
       
        # Integral term with anti-windup
        integral += error
        integral = clamp(integral, -INTEGRAL_MAX, INTEGRAL_MAX)
       
        # Derivative term
        derivative = error - previous_error
       
        # PID calculation
        power = (error * KP_DRIVE) + (integral * KI_DRIVE) + (derivative * KD_DRIVE)
       
        # Minimum power to overcome friction
        if abs(error) > 50:
            if power > 0:
                power = max(power, MIN_POWER)
            else:
                power = min(power, -MIN_POWER)
       
        power = clamp(power, -max_speed, max_speed)
       
        # Gyro correction for straight driving
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
    """Turn to an absolute heading using inertial sensor with PID control."""
    start_time = brain.timer.time(SECONDS)
    previous_error = 0
    integral = 0
   
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
       
        # Integral term with anti-windup
        integral += error
        integral = clamp(integral, -TURN_INTEGRAL_MAX, TURN_INTEGRAL_MAX)
       
        # Derivative term
        derivative = error - previous_error
       
        # PID calculation
        power = (error * KP_INERTIAL) + (integral * KI_INERTIAL) + (derivative * KD_INERTIAL)
       
        # Minimum power to overcome friction
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

#  CURVED MOVEMENT FUNCTION

def move_curve(distance_cm, target_heading, speed=MAX_POWER, timeout_sec=5.0):
    """
    Args:
        distance_cm: Distance to travel in millimeters
        target_heading: Absolute heading to reach by the end (0-360 degrees)
        speed: Maximum speed percentage (default 80%)
        timeout_sec: Timeout in seconds (default 5.0)
    """
    reset_drive_encoders()
   
    initial_heading = inertial.heading(DEGREES)
    target_degrees = cm_to_degrees(distance_cm)
   
    heading_change = target_heading - initial_heading
    if heading_change > 180:
        heading_change -= 360
    elif heading_change < -180:
        heading_change += 360
   
    start_time = brain.timer.time(SECONDS)
    previous_error = 0
    integral = 0
   
    while True:
        current_position = get_average_encoder_value()
        distance_error = target_degrees - current_position
       
        if abs(distance_error) < 10:
            break
       
        if (brain.timer.time(SECONDS) - start_time) > timeout_sec:
            break
       
        progress = current_position / target_degrees if target_degrees != 0 else 0
        progress = clamp(progress, 0, 1)
       
        desired_heading = (initial_heading + (heading_change * progress)) % 360
       
        current_heading = inertial.heading(DEGREES)
        heading_error = desired_heading - current_heading
       
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
       
        integral += distance_error
        integral = clamp(integral, -INTEGRAL_MAX, INTEGRAL_MAX)
        derivative = distance_error - previous_error
       
        forward_power = (distance_error * KP_DRIVE) + (integral * KI_DRIVE) + (derivative * KD_DRIVE)
       
        if abs(distance_error) > 50:
            if forward_power > 0:
                forward_power = max(forward_power, MIN_POWER)
            else:
                forward_power = min(forward_power, -MIN_POWER)
       
        forward_power = clamp(forward_power, -speed, speed)
       
        turn_adjustment = heading_error * 0.8
        turn_adjustment = clamp(turn_adjustment, -40, 40)
       
        left_power = forward_power + turn_adjustment
        right_power = forward_power - turn_adjustment
       
        left_drive.spin(FORWARD, left_power, PERCENT)
        right_drive.spin(FORWARD, right_power, PERCENT)
       
        previous_error = distance_error
        wait(20, MSEC)
   
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)

def move_curve_relative(distance_cm, heading_change, speed=MAX_POWER, timeout_sec=5.0):
    """
   
    Args:
        distance_cm: Distance to travel in millimeters
        heading_change: Degrees to turn during the move (positive = right, negative = left)
        speed: Maximum speed percentage (default 80%)
        timeout_sec: Timeout in seconds (default 5.0)
    """
    current_heading = inertial.heading(DEGREES)
    target_heading = (current_heading + heading_change) % 360
    move_curve(distance_cm, target_heading, speed, timeout_sec)

#  HIGH-LEVEL AUTONOMOUS FUNCTIONS

def move_cm(cm, speed=MAX_POWER, timeout_sec=5.0,):
    """
    Move forward or backward a specified distance in millimeters.
   
    Args:
        cm: Distance in millimeters (positive = forward, negative = backward)
        speed: Maximum speed percentage (default 80%)
        timeout_sec: Timeout in seconds (default 5.0)
    """
    degrees = cm_to_degrees(cm)
    drive_forward_pid(degrees, speed, timeout_sec)


def turn_right(degrees, timeout_sec=3.0):
    """Turn right relative to current heading."""
    turn_relative(degrees, timeout_sec)

def turn_left(degrees, timeout_sec=3.0):
    """Turn left relative to current heading."""
    turn_relative(-degrees, timeout_sec)

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

#  AUTONOMOUS ROUTINES

def autonomous_logo():
    move_curve_relative(35.15, -138, 32)
    intake_blue_motor(-100)
    move_cm(32, 35, timeout_sec=1.55)
    stop_intakeB()
    move_cm(-4.5, 45)
    intake_blue_motor(100,1.2)
    move_cm(-20, 55)
    move_curve_relative(-26, -138, 45)
    piston_H(True)
    move_cm(39,40,1.3)
    intake_blue_motor(100, .3)
    intake_blue_motor(-100)
    move_cm(10, 0, 2.3)
    move_curve_relative(-30, -25, 55,1.6)
    move_cm(-2, 40, 1)
    intake_both(-100,10)

def autonomous_mogo(): 
    intake_blue_motor(-100)
    move_curve_relative(42, -40, 40)
    move_curve_relative(-19, -115, 30)
    move_cm(-4, 40)
    intake_blue_motor(0)
    piston_G(True)
    intake_both(-100)
    wait(2, SECONDS)
    piston_G(False)
    intake_both(0)
    piston_H(True)
    move_curve_relative(62, -6.3, 44, 4)
    intake_blue_motor(100, .2)
    intake_blue_motor(-100)
    wait(1.8, SECONDS)
    intake_blue_motor(0)
    move_curve_relative(-27, -59, 50, 1.6)
    move_cm(-8, 40, 1)
    intake_both(-100,10)
    wait(2, SECONDS)
    intake_both(0)

def prog():
    move_curve_relative(42, 40, 40)
    intake_blue_motor(0)
    move_curve_relative(-19, 115, 30)
    move_cm(-4, 40)
    piston_G(True)
    intake_both(-100)
    wait(2, SECONDS)
    piston_G(False)
    intake_both(0)
    piston_H(True)
    move_curve_relative(62, 6.7, 40, 4)
    intake_blue_motor(100, .25)
    intake_blue_motor(-100)
    wait(2.4, SECONDS)
    intake_blue_motor(0)
    move_curve_relative(-26, 57, 50, 1.6)
    move_cm(-10, 40, 1)
    intake_both(-100,10)
    wait(2.4, SECONDS)
    intake_both(0)
    move_cm(-23, 40, 1)
    turn_right(90)
    intake_blue_motor(-100)
    move_cm(48, 40, 3)
    turn_left(45)
    move_cm(23, 20)
    intake_blue_motor(100)
    wait(1, SECONDS)
    intake_blue_motor(0)
    turn_right(90)
    move_cm(35, 30)
    turn_left(90)
    move_cm(20, 60)

def testing():
    intake_blue_motor(-100)
    move_curve(120, 13, 50)
    wait(0.3, SECONDS)
    intake_blue_motor(0)
    turn_left(69)
    move_cm(18, 30)
    intake_blue_motor(90)
    wait(1.3, SECONDS)
    intake_blue_motor(0)
    turn_left(-2)
    move_cm( -116.5+24, 60)
    piston_H(False)
    piston_H(True)
    turn_right(235)
    move_cm(60, 60)

def testing2():
    intake_blue_motor(-100)
    move_curve(113, -20, 50)
    wait(0.3, SECONDS)
    intake_blue_motor(0)
    turn_left(90+15)
    move_cm(-39, 40)
    intake_blue_motor(100)
    
#  MAIN AUTONOMOUS
def autonomous():
    testing2()

#  DRIVER CONTROL
def user_control():
    global previous_left_speed, previous_right_speed
    global mg_piston_state, ds_piston_state, ml_piston_state
    global button_a_was_pressed, button_x_was_pressed, button_b_was_pressed
   
    brain.screen.clear_screen()
    brain.screen.print("Driver Control")
    brain.screen.new_line()
    controller.screen.print("you've got this! -J")
    while True:
        #  DRIVE CONTROL
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
       
        #  INTAKE CONTROL
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
       
        #  PISTON CONTROL
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

#  MAIN PROGRAM
comp = Competition(user_control, autonomous)

