# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       jordanawilkes                                                #
# 	Created:      11/5/2025, 9:50:54 PM                                        #
# 	Description:  V5 project with PID control and custom brake settings        #
#                 FIXED VERSION - corrected PID, brake logic, and pistons      #
#                                                                              #
# ---------------------------------------------------------------------------- #
# Library imports
from vex import *

# === CONSTANTS ===
DEADBAND = 5
INTAKE_SPEED = 100
DRIVE_UPDATE_RATE = 20

# Brake Settings
DRIVE_BRAKE_STRENGTH_STRONG = 0.7  # Stronger brake option (0.0 = coast, 1.0 = full brake)
DRIVE_BRAKE_STRENGTH_WEAK = 0.4    # Weaker brake option for testing
ACTIVE_DRIVE_BRAKE = DRIVE_BRAKE_STRENGTH_STRONG  # Driver prefers stronger

# PID Constants (tune these values for your robot)
DRIVE_KP = 0.5      # Proportional gain for driving straight
DRIVE_KI = 0.0      # Integral gain
DRIVE_KD = 0.1      # Derivative gain

TURN_KP = 0.6       # Proportional gain for turning
TURN_KI = 0.0       # Integral gain
TURN_KD = 0.15      # Derivative gain

# === DEVICE CONFIGURATION ===
brain = Brain()
controller = Controller(PRIMARY)

# Pneumatic Pistons (DigitalOut for single solenoids)
mg_piston = DigitalOut(brain.three_wire_port.g)  # Button A, medium goal
ds_piston = DigitalOut(brain.three_wire_port.f)  # Button X, de-score
ml_piston = DigitalOut(brain.three_wire_port.h)  # Button B, match load

# Drivetrain motors
left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_motor_c = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_a = Motor(Ports.PORT15, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)
right_motor_c = Motor(Ports.PORT16, GearSetting.RATIO_18_1, False)

left_drive = MotorGroup(left_motor_a, left_motor_b, left_motor_c)
right_drive = MotorGroup(right_motor_a, right_motor_b, right_motor_c)

# Configure brake modes - COAST for custom brake to work properly
left_drive.set_stopping(COAST)
right_drive.set_stopping(COAST)

# Intake motors
intake_blue = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
intake_small = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)

# Inertial sensor for PID
inertial = Inertial(Ports.PORT10)

# === CUSTOM BRAKE VARIABLES ===
previous_left_speed = 0
previous_right_speed = 0

# === PID CONTROLLER CLASS ===
class PIDController:
    """PID Controller for precise robot movements."""
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0
        self.output_limit = 100  # Max motor speed percentage
        
    def calculate(self, setpoint, current_value, dt=0.02):
        """
        Calculate PID output.
        
        Args:
            setpoint: Target value
            current_value: Current measured value
            dt: Time delta in seconds (default 20ms = 0.02s)
        
        Returns:
            PID output value
        """
        # Calculate error
        error = setpoint - current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * dt
        # Limit integral to prevent windup
        max_integral = 50 / max(self.ki, 0.01)  # Adaptive limit based on Ki
        self.integral = max(-max_integral, min(max_integral, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.previous_error) / dt
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Limit output
        output = max(-self.output_limit, min(self.output_limit, output))
        
        # Store error for next iteration
        self.previous_error = error
        
        return output
    
    def reset(self):
        """Reset PID controller state."""
        self.integral = 0
        self.previous_error = 0

# === HELPER FUNCTIONS ===
def apply_deadband(value, deadband=DEADBAND):
    """Apply deadband to joystick input to reduce drift."""
    return 0 if abs(value) < deadband else value

def apply_custom_brake(current_speed, previous_speed, brake_strength):
    """
    Apply custom brake between BRAKE and COAST modes.
    
    Args:
        current_speed: Commanded speed from driver (0 = wants to stop)
        previous_speed: Speed from previous loop iteration
        brake_strength: 0.0 = coast, 1.0 = full brake, values between = custom
    
    Returns:
        Adjusted speed with braking applied
    """
    # If driver is commanding movement, return that speed
    if abs(current_speed) > DEADBAND:
        return current_speed
    
    # Driver wants to stop - apply proportional braking
    # Reduce speed based on brake strength
    brake_factor = 1.0 - brake_strength
    new_speed = previous_speed * brake_factor
    
    # Stop completely when speed gets very low
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

# === PID MOVEMENT FUNCTIONS ===
def drive_straight_pid(distance_degrees, speed=50):
    """
    Drive straight using PID to maintain equal motor positions.
    
    Args:
        distance_degrees: Distance to travel in motor degrees (positive = forward, negative = backward)
        speed: Base forward speed (0-100)
    """
    # Create PID controller for maintaining straight line
    pid = PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD)
    
    # Reset motor encoders
    left_drive.set_position(0, DEGREES)
    right_drive.set_position(0, DEGREES)
    
    # Determine direction and target
    direction = FORWARD if distance_degrees > 0 else REVERSE
    target_position = abs(distance_degrees)
    tolerance = 10  # degrees
    
    while True:
        # Get current positions (absolute values)
        left_pos = abs(left_drive.position(DEGREES))
        right_pos = abs(right_drive.position(DEGREES))
        
        # Average position
        avg_pos = (left_pos + right_pos) / 2
        
        # Check if target reached
        if avg_pos >= target_position - tolerance:
            break
        
        # Calculate correction to keep robot straight
        position_diff = right_pos - left_pos
        correction = pid.calculate(0, position_diff)
        
        # Apply speeds with correction
        left_speed = speed - correction
        right_speed = speed + correction
        
        # Clamp speeds to valid range
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        left_drive.spin(direction, abs(left_speed), PERCENT)
        right_drive.spin(direction, abs(right_speed), PERCENT)
        
        wait(DRIVE_UPDATE_RATE, MSEC)
    
    # Stop motors
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)

def turn_to_heading(target_heading, timeout_ms=3000):
    """
    Turn to an absolute heading using PID and inertial sensor.
    
    Args:
        target_heading: Target heading in degrees (0-360)
        timeout_ms: Maximum time to attempt turn in milliseconds
    
    Returns:
        True if target reached, False if timeout
    """
    pid = PIDController(TURN_KP, TURN_KI, TURN_KD)
    
    tolerance = 2  # degrees
    settled_count = 0
    settled_threshold = 3  # Need to be within tolerance for this many loops
    start_time = brain.timer.time(MSEC)
    
    while brain.timer.time(MSEC) - start_time < timeout_ms:
        current_heading = inertial.heading(DEGREES)
        
        # Calculate shortest path to target (handles 0/360 wraparound)
        error = target_heading - current_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        
        # Check if we're at target
        if abs(error) < tolerance:
            settled_count += 1
            if settled_count >= settled_threshold:
                left_drive.stop(BRAKE)
                right_drive.stop(BRAKE)
                return True
        else:
            settled_count = 0
        
        # FIXED: Calculate turn power correctly
        # Positive error means we need to turn right (clockwise)
        # This makes left side go forward, right side go backward
        turn_power = pid.calculate(target_heading, current_heading)
        
        # Apply turning (tank turn)
        left_drive.spin(FORWARD, turn_power, PERCENT)
        right_drive.spin(FORWARD, -turn_power, PERCENT)
        
        wait(DRIVE_UPDATE_RATE, MSEC)
    
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    return False

def move_distance(distance_inches, speed=50, timeout_ms=5000):
    """
    Move forward/backward a specific distance using motor encoders.
    
    Args:
        distance_inches: Distance to travel in inches (positive = forward, negative = backward)
        speed: Base speed percentage (0-100)
        timeout_ms: Maximum time to attempt move in milliseconds
    
    Returns:
        True if target reached, False if timeout
    """
    # Robot specifications
    WHEEL_DIAMETER = 3.25  # inches
    GEAR_RATIO = 1.67      # motor rotations : wheel rotations
    
    # Calculate wheel circumference
    wheel_circumference = WHEEL_DIAMETER * 3.14159265359
    
    # Calculate required wheel rotations
    wheel_rotations = distance_inches / wheel_circumference
    
    # Calculate required motor degrees (accounting for gear ratio)
    motor_degrees = wheel_rotations * 360 * GEAR_RATIO
    
    # Create PID controller for driving straight
    pid = PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD)
    
    # Reset motor encoders
    left_drive.set_position(0, DEGREES)
    right_drive.set_position(0, DEGREES)
    
    # Determine direction
    direction = FORWARD if motor_degrees > 0 else REVERSE
    target_position = abs(motor_degrees)
    speed = abs(speed)
    
    tolerance = 15  # degrees
    start_time = brain.timer.time(MSEC)
    
    while brain.timer.time(MSEC) - start_time < timeout_ms:
        # Get current positions (absolute values)
        left_pos = abs(left_drive.position(DEGREES))
        right_pos = abs(right_drive.position(DEGREES))
        
        # Average position
        avg_pos = (left_pos + right_pos) / 2
        
        # Check if target reached
        if avg_pos >= target_position - tolerance:
            left_drive.stop(BRAKE)
            right_drive.stop(BRAKE)
            return True
        
        # Calculate correction to keep robot straight
        position_diff = right_pos - left_pos
        correction = pid.calculate(0, position_diff)
        
        # Apply speeds with correction
        left_speed = speed - correction
        right_speed = speed + correction
        
        # Clamp speeds to valid range
        left_speed = max(10, min(100, abs(left_speed)))
        right_speed = max(10, min(100, abs(right_speed)))
        
        left_drive.spin(direction, left_speed, PERCENT)
        right_drive.spin(direction, right_speed, PERCENT)
        
        wait(DRIVE_UPDATE_RATE, MSEC)
    
    # Timeout - stop motors
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    return False

# === AUTONOMOUS ===
def autonomous():
    """Autonomous mode - runs for 15 seconds at match start."""
    brain.screen.clear_screen()
    brain.screen.print("Autonomous Mode")
    
    # Calibrate inertial sensor at start
    brain.screen.new_line()
    brain.screen.print("Calibrating...")
    inertial.calibrate()
    while inertial.is_calibrating():
        wait(50, MSEC)
    brain.screen.new_line()
    brain.screen.print("Ready!")
    
    # Example autonomous routine using new functions
    
    # Move forward 24 inches at 60% speed
    move_distance(24, speed=60)
    wait(300, MSEC)
    
    # Turn to heading 90 degrees
    turn_to_heading(90)
    wait(300, MSEC)
    
    # Move backward 12 inches
    move_distance(-12, speed=50)
    wait(300, MSEC)
    
    # Turn to heading 180 degrees
    turn_to_heading(180)
    
    # Old encoder-based function still available:
    # drive_straight_pid(1000, speed=50)

# === DRIVER CONTROL ===
def user_control():
    """Driver control mode - runs after autonomous."""
    global previous_left_speed, previous_right_speed
    
    brain.screen.clear_screen()
    brain.screen.print("Driver Control")
    brain.screen.new_line()
    brain.screen.print("Brake: " + str(ACTIVE_DRIVE_BRAKE))
    
    # Calibrate inertial if not already done (in case driver control starts first)
    if not inertial.installed():
        pass  # Sensor not connected
    elif inertial.is_calibrating():
        pass  # Already calibrating
    else:
        # Check if it needs calibration by trying to read heading
        try:
            test_heading = inertial.heading()
        except:
            inertial.calibrate()
    
    while True:
        # === DRIVE CONTROL (ARCADE STYLE WITH CUSTOM BRAKE) ===
        forward = apply_deadband(controller.axis1.position())
        turn = apply_deadband(controller.axis3.position())
        
        # Calculate desired speeds (arcade drive)
        desired_left_speed = forward - turn
        desired_right_speed = forward + turn
        
        # FIXED: Apply custom brake properly
        # Check if driver wants to stop (both sticks in deadband)
        if abs(forward) < DEADBAND and abs(turn) < DEADBAND:
            # Driver released both sticks - apply custom brake
            left_speed = apply_custom_brake(0, previous_left_speed, ACTIVE_DRIVE_BRAKE)
            right_speed = apply_custom_brake(0, previous_right_speed, ACTIVE_DRIVE_BRAKE)
        else:
            # Driver is actively driving - use commanded speeds
            left_speed = desired_left_speed
            right_speed = desired_right_speed
        
        # FIXED: Always use spin() to allow custom brake to work
        # Never use stop() in driver control with custom brake
        left_drive.spin(FORWARD, left_speed, PERCENT)
        right_drive.spin(FORWARD, right_speed, PERCENT)
        
        # Store speeds for next iteration
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
        # Medium Goal Piston - Button A
        if controller.buttonA.pressing():
            mg_piston.set(True)
        else:
            mg_piston.set(False)
        
        # De-score Piston - Button X
        if controller.buttonX.pressing():
            ds_piston.set(True)
        else:
            ds_piston.set(False)
        
        # Match Load Piston - Button B
        if controller.buttonB.pressing():
            ml_piston.set(True)
        else:
            ml_piston.set(False)
        
        wait(DRIVE_UPDATE_RATE, MSEC)

# === MAIN PROGRAM ===
# Create competition instance
comp = Competition(user_control, autonomous)

# Display startup message
brain.screen.clear_screen()
brain.screen.print("Program Started")
brain.screen.new_line()
brain.screen.print("Ready for Competition")
brain.screen.new_line()
brain.screen.print("PID Enabled")
brain.screen.new_line()
brain.screen.print("Custom Brake: " + str(ACTIVE_DRIVE_BRAKE))
