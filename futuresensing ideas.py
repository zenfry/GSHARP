# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       jordanawilkes                                                #
# 	Created:      11/5/2025, 9:50:54 PM                                        #
# 	Description:  V5 project with Advanced Sensing                             #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *
import math

# === CONSTANTS ===
DEADBAND = 5
INTAKE_SPEED = 100
DRIVE_UPDATE_RATE = 20

# Autonomous constants
AUTO_DRIVE_SPEED = 50
AUTO_TURN_SPEED = 30
DISTANCE_THRESHOLD = 50  # mm for object detection
LINE_THRESHOLD = 50      # % for line following

# === DEVICE CONFIGURATION ===
brain = Brain()
controller = Controller(PRIMARY)

# Drivetrain motors
left_motor_a = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT15, GearSetting.RATIO_18_1, False)
left_motor_c = Motor(Ports.PORT16, GearSetting.RATIO_18_1, True)
right_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
right_motor_c = Motor(Ports.PORT13, GearSetting.RATIO_18_1, True)

left_drive = MotorGroup(left_motor_a, left_motor_b, left_motor_c)
right_drive = MotorGroup(right_motor_a, right_motor_b, right_motor_c)

# Intake motors
intake_blue = Motor(Ports.PORT6, GearSetting.RATIO_18_1, True)
intake_small = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)

# Pneumatics
piston_match_load = Pneumatic(brain.three_wire_port.a)
piston_medium_goal = Pneumatic(brain.three_wire_port.b)

# === SENSORS ===
# Vision Sensor - Object detection and color tracking
vision_sensor = Vision(Ports.PORT1)
# Configure vision signatures (adjust RGB values for your game pieces)
BLUE_SIG = Signature(1, -3500, -2500, -3000, 9000, 12000, 10500, 3.0, 0)
RED_SIG = Signature(2, 7000, 9000, 8000, -1500, -500, -1000, 3.0, 0)
vision_sensor.set_signature(1, BLUE_SIG)
vision_sensor.set_signature(2, RED_SIG)

# Distance Sensor - Obstacle detection
distance_sensor = Distance(Ports.PORT2)

# Optical Sensor - Color and proximity detection
optical_sensor = Optical(Ports.PORT3)

# Inertial Sensor - Gyro for precise turns and heading
inertial_sensor = Inertial(Ports.PORT4)

# GPS Sensor - Position tracking on field
gps_sensor = Gps(Ports.PORT5, 0, 0, MM, 0)  # Adjust offset from center

# Line Trackers (3-wire sensors)
line_left = Line(brain.three_wire_port.c)
line_center = Line(brain.three_wire_port.d)
line_right = Line(brain.three_wire_port.e)

# Limit Switches - Mechanical feedback
limit_intake = Limit(brain.three_wire_port.f)
limit_back = Limit(brain.three_wire_port.g)

# Bumper Switch - Collision detection
bumper_front = Bumper(brain.three_wire_port.h)

# Rotation Sensor - Precise mechanism tracking
rotation_sensor = Rotation(Ports.PORT8)

# === SENSOR DATA CLASS ===
class SensorData:
    """Stores and manages all sensor readings."""
    def __init__(self):
        self.distance = 0
        self.heading = 0
        self.gps_x = 0
        self.gps_y = 0
        self.optical_hue = 0
        self.optical_brightness = 0
        self.line_left_value = 0
        self.line_center_value = 0
        self.line_right_value = 0
        self.vision_objects = []
        self.rotation_angle = 0
    
    def update(self):
        """Update all sensor readings."""
        self.distance = distance_sensor.object_distance(MM)
        self.heading = inertial_sensor.heading()
        self.gps_x = gps_sensor.x_position(MM)
        self.gps_y = gps_sensor.y_position(MM)
        self.optical_hue = optical_sensor.hue()
        self.optical_brightness = optical_sensor.brightness()
        self.line_left_value = line_left.reflectivity()
        self.line_center_value = line_center.reflectivity()
        self.line_right_value = line_right.reflectivity()
        self.vision_objects = vision_sensor.take_snapshot(BLUE_SIG)
        self.rotation_angle = rotation_sensor.position(DEGREES)
    
    def display_on_brain(self):
        """Display sensor data on brain screen."""
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print("=== SENSOR DATA ===")
        brain.screen.set_cursor(2, 1)
        brain.screen.print("Dist: %d mm" % self.distance)
        brain.screen.set_cursor(3, 1)
        brain.screen.print("Head: %.1f deg" % self.heading)
        brain.screen.set_cursor(4, 1)
        brain.screen.print("GPS: (%.0f, %.0f)" % (self.gps_x, self.gps_y))
        brain.screen.set_cursor(5, 1)
        brain.screen.print("Hue: %d" % self.optical_hue)
        brain.screen.set_cursor(6, 1)
        brain.screen.print("Line: %d %d %d" % (self.line_left_value, 
                                                self.line_center_value, 
                                                self.line_right_value))

sensor_data = SensorData()

# === HELPER FUNCTIONS ===
def apply_deadband(value, deadband=DEADBAND):
    """Apply deadband to joystick input to reduce drift."""
    return 0 if abs(value) < deadband else value

def toggle_pneumatic(piston):
    """Toggle pneumatic state (open/closed)."""
    if piston.value():
        piston.close()
    else:
        piston.open()

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

# === ADVANCED SENSOR FUNCTIONS ===
def calibrate_sensors():
    """Calibrate sensors at startup."""
    brain.screen.clear_screen()
    brain.screen.print("Calibrating Inertial...")
    inertial_sensor.calibrate()
    while inertial_sensor.is_calibrating():
        wait(50, MSEC)
    brain.screen.new_line()
    brain.screen.print("Calibration Complete!")
    wait(500, MSEC)

def detect_object_with_vision():
    """Detect and return largest object seen by vision sensor."""
    vision_sensor.take_snapshot(BLUE_SIG)
    if vision_sensor.object_count() > 0:
        largest = vision_sensor.largest_object()
        return {
            'exists': True,
            'x': largest.centerX,
            'y': largest.centerY,
            'width': largest.width,
            'height': largest.height
        }
    return {'exists': False}

def align_to_vision_target():
    """Use vision sensor to align robot to target."""
    obj = detect_object_with_vision()
    if obj['exists']:
        # Center is at x=158 for VEX vision sensor
        error = obj['x'] - 158
        if abs(error) > 20:  # Tolerance
            turn_speed = error * 0.3  # Proportional control
            left_drive.spin(FORWARD, turn_speed, PERCENT)
            right_drive.spin(REVERSE, turn_speed, PERCENT)
            return False
        else:
            left_drive.stop()
            right_drive.stop()
            return True
    return False

def drive_to_distance(target_mm):
    """Drive forward/backward to reach target distance from obstacle."""
    current_distance = distance_sensor.object_distance(MM)
    error = current_distance - target_mm
    
    if abs(error) > 20:
        speed = max(min(error * 0.3, 50), -50)  # Proportional with limits
        left_drive.spin(FORWARD, speed, PERCENT)
        right_drive.spin(FORWARD, speed, PERCENT)
        return False
    else:
        left_drive.stop()
        right_drive.stop()
        return True

def turn_to_heading(target_degrees):
    """Turn to absolute heading using inertial sensor."""
    current_heading = inertial_sensor.heading()
    error = target_degrees - current_heading
    
    # Normalize error to -180 to 180
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    
    if abs(error) > 2:  # 2 degree tolerance
        turn_speed = max(min(error * 0.5, AUTO_TURN_SPEED), -AUTO_TURN_SPEED)
        left_drive.spin(FORWARD, turn_speed, PERCENT)
        right_drive.spin(REVERSE, turn_speed, PERCENT)
        return False
    else:
        left_drive.stop()
        right_drive.stop()
        return True

def follow_line():
    """Follow line using three line trackers with PID-like control."""
    left_val = line_left.reflectivity()
    center_val = line_center.reflectivity()
    right_val = line_right.reflectivity()
    
    # Calculate line position (-1 = left, 0 = center, 1 = right)
    if center_val < LINE_THRESHOLD:
        position = 0
    elif left_val < LINE_THRESHOLD:
        position = -1
    elif right_val < LINE_THRESHOLD:
        position = 1
    else:
        # Lost line
        left_drive.stop()
        right_drive.stop()
        return False
    
    # Proportional steering
    correction = position * 20
    left_drive.spin(FORWARD, AUTO_DRIVE_SPEED - correction, PERCENT)
    right_drive.spin(FORWARD, AUTO_DRIVE_SPEED + correction, PERCENT)
    return True

def drive_to_gps_position(target_x, target_y):
    """Drive to specific GPS coordinates."""
    current_x = gps_sensor.x_position(MM)
    current_y = gps_sensor.y_position(MM)
    
    # Calculate distance and angle to target
    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.sqrt(dx*dx + dy*dy)
    target_angle = math.degrees(math.atan2(dy, dx))
    
    if distance > 100:  # 100mm tolerance
        # First turn to target
        if not turn_to_heading(target_angle):
            return False
        # Then drive forward
        left_drive.spin(FORWARD, AUTO_DRIVE_SPEED, PERCENT)
        right_drive.spin(FORWARD, AUTO_DRIVE_SPEED, PERCENT)
        return False
    else:
        left_drive.stop()
        right_drive.stop()
        return True

def check_game_piece_color():
    """Use optical sensor to identify game piece color."""
    hue = optical_sensor.hue()
    brightness = optical_sensor.brightness()
    
    if brightness < 10:
        return "NONE"
    elif 200 < hue < 240:  # Blue range
        return "BLUE"
    elif hue < 20 or hue > 340:  # Red range
        return "RED"
    else:
        return "UNKNOWN"

def smart_intake():
    """Intelligent intake that detects when object is collected."""
    if limit_intake.pressing():
        # Object detected by limit switch
        control_intake(0, 0)
        return True
    else:
        control_intake(INTAKE_SPEED, INTAKE_SPEED)
        return False

# === AUTONOMOUS ===
def autonomous():
    """Autonomous mode with sensor-based navigation."""
    brain.screen.clear_screen()
    brain.screen.print("Autonomous Mode")
    
    # Example autonomous routine using sensors:
    
    # 1. Drive forward until object detected
    while distance_sensor.object_distance(MM) > 300:
        left_drive.spin(FORWARD, AUTO_DRIVE_SPEED, PERCENT)
        right_drive.spin(FORWARD, AUTO_DRIVE_SPEED, PERCENT)
        wait(20, MSEC)
    left_drive.stop()
    right_drive.stop()
    
    # 2. Turn 90 degrees using inertial
    while not turn_to_heading(90):
        wait(20, MSEC)
    
    # 3. Align to vision target
    timeout = 0
    while not align_to_vision_target() and timeout < 100:
        wait(20, MSEC)
        timeout += 1
    
    # 4. Drive to specific distance
    while not drive_to_distance(200):
        wait(20, MSEC)
    
    # 5. Activate intake until object collected
    while not smart_intake():
        wait(20, MSEC)

# === DRIVER CONTROL ===
def user_control():
    """Driver control mode with sensor feedback."""
    brain.screen.clear_screen()
    brain.screen.print("Driver Control")
    
    # Button state tracking
    r1_last_state = False
    r2_last_state = False
    x_last_state = False
    
    # Sensor display toggle
    show_sensors = False
    
    while True:
        # Update sensor data
        sensor_data.update()
        
        # Display sensors if enabled
        if show_sensors:
            sensor_data.display_on_brain()
        
        # === DRIVE CONTROL (ARCADE STYLE) ===
        forward = apply_deadband(controller.axis2.position())
        turn = apply_deadband(controller.axis1.position())
        
        left_speed = forward + turn
        right_speed = forward - turn
        
        left_drive.spin(FORWARD, left_speed, PERCENT)
        right_drive.spin(FORWARD, right_speed, PERCENT)
        
        # === INTAKE CONTROL WITH SMART FEATURES ===
        if controller.buttonL2.pressing():
            # Smart intake - stops when limit switch pressed
            if not smart_intake():
                control_intake(INTAKE_SPEED, INTAKE_SPEED)
        elif controller.buttonL1.pressing():
            control_intake(-INTAKE_SPEED, -INTAKE_SPEED)
        elif controller.buttonUp.pressing():
            control_intake(INTAKE_SPEED, 0)
        elif controller.buttonDown.pressing():
            control_intake(-INTAKE_SPEED, 0)
        else:
            control_intake(0, 0)
        
        # === PNEUMATIC CONTROL ===
        r1_current = controller.buttonR1.pressing()
        if r1_current and not r1_last_state:
            toggle_pneumatic(piston_match_load)
        r1_last_state = r1_current
        
        r2_current = controller.buttonR2.pressing()
        if r2_current and not r2_last_state:
            toggle_pneumatic(piston_medium_goal)
        r2_last_state = r2_current
        
        # === SENSOR DISPLAY TOGGLE ===
        x_current = controller.buttonX.pressing()
        if x_current and not x_last_state:
            show_sensors = not show_sensors
            if not show_sensors:
                brain.screen.clear_screen()
                brain.screen.print("Driver Control")
        x_last_state = x_current
        
        # === COLLISION WARNING ===
        if distance_sensor.object_distance(MM) < 100:
            controller.rumble(".")  # Short rumble for obstacle
        
        # === GAME PIECE COLOR FEEDBACK ===
        color = check_game_piece_color()
        if color == "BLUE":
            controller.rumble("--")  # Long rumble for blue
        elif color == "RED":
            controller.rumble(".-")  # Different pattern for red
        
        wait(DRIVE_UPDATE_RATE, MSEC)

# === MAIN PROGRAM ===
# Calibrate sensors before competition
calibrate_sensors()

# Create competition instance
comp = Competition(user_control, autonomous)

# Display startup message
brain.screen.clear_screen()
brain.screen.print("Advanced Sensing Ready")
brain.screen.new_line()
brain.screen.print("Press X for Sensor View")
