# GSHARP
Library Import and Constants
The program begins by importing the VEX library, which provides all the classes and functions needed to control VEX V5 hardware. Three constants are defined:

DEADBAND = 5 creates a 5% threshold zone around the joystick's center position to prevent motor jitter from controller drift
INTAKE_SPEED = 100 sets the default power level (as a percentage) for intake motors
DRIVE_UPDATE_RATE = 20 determines the control loop runs every 20 milliseconds, resulting in 50 updates per second

Device Configuration - Brain and Controller
The brain object represents the V5 robot brain, which is the central computer controlling everything. The controller object represents the driver's handheld controller used during matches.

Drivetrain Setup
The robot uses a six-motor drivetrain configuration with three motors on each side:

Left side motors:

All three motors (ports 14, 15, 16) use 18:1 gear ratio
Motors A and B are set to False (not reversed), while motor C is set to True (reversed) to ensure all motors spin the same direction relative to the robot
These are combined into a left_drive motor group 

Right side motors:

Three motors (ports 11, 12, 13) mirror the left side configuration
Motors A and B are not reversed, motor C is reversed
Combined into a right_drive motor group


Intake System
Two separate intake motors are configured:

intake_blue on port 6 is reversed and uses an 18:1 gear ratio
intake_small on port 7 is also reversed with an 18:1 gear ratio

Pneumatic Systems
Two pneumatic cylinders are connected to the brain's three-wire ports:

piston_match_load on port A 
piston_medium_goal on port B 


Helper Function: apply_deadband
This function takes a joystick value (ranging from -100 to 100) and applies a deadband filter:

If the absolute value of the input is less than the deadband threshold (5%), it returns 0
Otherwise, it returns the original value unchanged
This prevents motors from making tiny unwanted movements when the joystick is nearly centered but not perfectly still

Helper Function: toggle_pneumatic
This function switches a pneumatic piston's state:

It checks the current state using piston.value() which returns whether the piston is currently open
If currently open (True), it closes the piston
If currently closed (False), it opens the piston
This creates a simple on/off toggle behavior

Helper Function: control_intake
This function manages both intake motors simultaneously with individual speed control:

Takes two parameters: blue_speed and small_speed (can be positive, negative, or zero)
For each motor, if speed is 0, it stops the motor completely
If speed is non-zero, it determines direction (FORWARD for positive, REVERSE for negative) and spins at the absolute value of the speed
This allows independent control: one intake forward, one backward, both same direction, etc.

Autonomous Function
This mode runs automatically at the start of every match for exactly 15 seconds:

Clears the brain's screen and displays "Autonomous Mode"


User Control Function - Main Control Loop
This is where the driver takes manual control after autonomous ends.
Initial Setup:

Clears the screen and displays "Driver Control"
Creates two variables (r1_last_state and r2_last_state) to track whether R1 and R2 buttons were pressed in the previous loop iteration
This tracking enables edge detection for toggle functionality

Main Loop (runs continuously):
Drive Control (Arcade Style)

Reads the right joystick's vertical axis (axis2) for forward/backward movement
Reads the right joystick's horizontal axis (axis1) for turning
Both values are filtered through the deadband function

Arcade drive math:

Left side speed = forward + turn (when turning right, left side speeds up)
Right side speed = forward - turn (when turning right, right side slows down)


Both motor groups are commanded to spin with their calculated speeds

Intake Control System
The code implements a priority-based control scheme:
Priority 1 (L2 button): If L2 is pressed, both intakes spin forward at full speed
Priority 2 (L1 button): If L2 isn't pressed but L1 is, both intakes reverse at full speed 
Priority 3 (Up button): If neither shoulder button is pressed but Up is, only the blue intake spins forward - for selective intake control
Priority 4 (Down button): If none of the above but Down is pressed, only the blue intake reverses - for selective ejection
Default: If no intake buttons are pressed, both intakes stop completely
This priority system prevents conflicting commands (multiple buttons pressed simultaneously).

Pneumatic Control with Edge Detection

R1 Button (Match Load Piston):

Reads the current button state
Compares it to the previous state stored in r1_last_state
Only triggers the toggle if the button is currently pressed AND wasn't pressed in the last loop iteration
Updates r1_last_state for the next iteration
This edge detection ensures one button press = one toggle, not continuous toggling while held

R2 Button (Medium Goal Piston):

Identical logic to R1 but controls the medium goal piston
Separate state tracking allows independent operation

Loop Timing
The loop waits 20 milliseconds at the end of each iteration, creating a consistent 50Hz update rate. 
Main Program Execution
Competition Setup:

Creates a Competition object that manages the match flow
Links the user_control and autonomous functions so they run at the appropriate times
The competition object automatically handles switching between modes based on field control signals

Startup Display:

Clears the brain screen
Prints "Program Started" on the first line
Creates a new line
Prints "Ready for Competition"
