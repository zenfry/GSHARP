# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       jordanawilkes                                                #
# 	Created:      11/5/2025, 9:50:54 PM                                        #
# 	Description:  V5 project with PID control and custom brake settings       #
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

# Pistons
MG_piston = adi.DigitalOut('G') #Button A, medium goal
DS_piston = adi.DigitalOut('E') #Button X, de-score
ML_piston = adi.DigitalOut('H') #Button B, match load

# Drivetrain motors
left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_motor_c = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_a = Motor(Ports.PORT15, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)
right_motor_c = Motor(Ports.PORT16, GearSetting.RATIO_18_1, False)

left_drive = MotorGroup(left_motor_a, left_motor_b, left_motor_c)
right_drive = MotorGroup(right_motor_a, right_motor_b, right_motor_c)

# Configure brake modes
# Turn motors: BRAKE mode for precise control
left_drive.set_stopping(BRAKE)
right_drive.set_stopping(BRAKE)

# Intake motors
intake_blue = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
intake_small = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)

# Inertial sensor for PID (Add the port below)
# inertial = Inertial(Ports.PORT1)

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
        self.integral = max(-50, min(50, self.integral))
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
    left_drive.stop()
    right_drive.stop()

def turn_pid(target_degrees):
    """
    Turn to a specific angle using PID (requires inertial sensor).
    
    Args:
        target_degrees: Target angle in degrees (positive = right, negative = left)
    """
    # NOTE: Uncomment this function when using the intertial sensor
    '''
    pid = PIDController(TURN_KP, TURN_KI, TURN_KD)
    
    inertial.set_heading(0, DEGREES)
    tolerance = 2  # degrees
    
    while True:
        current_heading = inertial.heading(DEGREES)
        
        # Normalize angle to -180 to 180
        if current_heading > 180:
            current_heading -= 360
        
        if abs(current_heading - target_degrees) < tolerance:
            break
        
        # Calculate turn power
        turn_power = pid.calculate(target_degrees, current_heading)
        
        # Apply opposite speeds to turn
        left_drive.spin(FORWARD, turn_power, PERCENT)
        right_drive.spin(FORWARD, -turn_power, PERCENT)
        
        wait(DRIVE_UPDATE_RATE, MSEC)
    
    left_drive.stop()
    right_drive.stop()
    '''
    pass

# === AUTONOMOUS ===
def autonomous():
    """Autonomous mode - runs for 15 seconds at match start."""
    brain.screen.clear_screen()
    brain.screen.print("Autonomous Mode")
    
    # Example autonomous routine using PID
    # Drive forward 1000 degrees
    drive_straight_pid(1000, speed=50)
    wait(500, MSEC)
    
    # Drive backward
    drive_straight_pid(-1000, speed=50)
    
    # Turn (requires inertial sensor)
    # turn_pid(90)  # Turn 90 degrees right

# === DRIVER CONTROL ===
def user_control():
    """Driver control mode - runs after autonomous."""
    global previous_left_speed, previous_right_speed
    
    brain.screen.clear_screen()
    brain.screen.print("Driver Control")
    brain.screen.new_line()
    brain.screen.print("Brake: " + str(ACTIVE_DRIVE_BRAKE))
    
    while True:
        # === DRIVE CONTROL (ARCADE STYLE WITH CUSTOM BRAKE) ===
        forward = apply_deadband(controller.axis3.position())
        turn = apply_deadband(controller.axis1.position())
        
        # Calculate desired speeds (arcade drive)
        desired_left_speed = forward + turn
        desired_right_speed = forward - turn
        
        # Apply custom brake when driver releases sticks (for drive only, not turns)
        # Only apply custom brake to forward/backward motion
        if abs(forward) < DEADBAND and abs(turn) >= DEADBAND:
            # Driver is turning only - use standard BRAKE mode (already set)
            left_speed = desired_left_speed
            right_speed = desired_right_speed
        elif abs(forward) < DEADBAND and abs(turn) < DEADBAND:
            # Driver released both sticks - apply custom brake to forward motion
            left_speed = apply_custom_brake(desired_left_speed, previous_left_speed, ACTIVE_DRIVE_BRAKE)
            right_speed = apply_custom_brake(desired_right_speed, previous_right_speed, ACTIVE_DRIVE_BRAKE)
        else:
            # Driver is actively driving - use commanded speeds
            left_speed = desired_left_speed
            right_speed = desired_right_speed
        
        # Send speeds to motors
        if abs(left_speed) < 1:
            left_drive.stop(BRAKE)  # Use BRAKE for turns
        else:
            left_drive.spin(FORWARD, left_speed, PERCENT)
        
        if abs(right_speed) < 1:
            right_drive.stop(BRAKE)  # Use BRAKE for turns
        else:
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
            MG_piston.set(True)
        else:
            MG_piston.set(False)
        
        # De-score Piston - Button X
        if controller.buttonX.pressing():
            DS_piston.set(True)
        else:
            DS_piston.set(False)
        
        # Match Load Piston - Button B
        if controller.buttonB.pressing():
            ML_piston.set(True)
        else:
            ML_piston.set(False)
        
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
