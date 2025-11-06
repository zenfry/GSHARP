# GSHARP - VEX Robotics Control System

A comprehensive driver control program for VEX V5 robotics competition, featuring arcade-style driving, dual intake system, and pneumatic controls.

## Controller Layout

### Drive Controls
- **Right Joystick (Vertical)** - Forward/Backward movement
- **Right Joystick (Horizontal)** - Left/Right turning
- *Arcade drive style with 5% deadband to prevent drift*

### Intake Controls
| Button | Function | Description |
|--------|----------|-------------|
| **L2** | Both Intakes Forward | Primary intake button - picks up game pieces at 100% speed |
| **L1** | Both Intakes Reverse | Ejects or unjams rings/game pieces at 100% speed |
| **D-Pad UP** | Blue Intake Forward | Selective control for blue intake only |
| **D-Pad DOWN** | Blue Intake Reverse | Reverse blue intake independently |

### Pneumatic Controls
| Button | Function | Type |
|--------|----------|------|
| **R1** | Match Load Piston | Toggle (press to open/close) |
| **R2** | Medium Goal Piston | Toggle (press to open/close) |

*Note: Pneumatic toggles use edge detection - press once to activate, press again to deactivate. Won't rapid-fire if held.*

### Available Buttons
The following buttons are currently unused and available for future features:
- D-Pad LEFT
- D-Pad RIGHT
- Buttons A, B, X, Y
- Left Joystick (all axes)

## Hardware Configuration

### Drivetrain
**Six-motor tank drive configuration:**
- **Left Side:** Ports 14, 15, 16 (18:1 gear ratio)
- **Right Side:** Ports 11, 12, 13 (18:1 gear ratio)
- Motor orientation configured for synchronized movement

### Intake System
- **Blue Intake:** Port 6 (18:1 gear ratio, reversed)
- **Small Intake:** Port 7 (18:1 gear ratio, reversed)

### Pneumatics
- **Match Load Piston:** 3-wire Port A
- **Medium Goal Piston:** 3-wire Port B

## Technical Specifications

- **Control Loop Rate:** 50Hz (updates every 20ms)
- **Joystick Deadband:** 5% threshold
- **Intake Speed:** 100% default power
- **Priority System:** L2 > L1 > D-Pad UP > D-Pad DOWN

## Features

### Arcade Drive System
Intuitive single-joystick control combining forward/backward movement with turning:
```
Left Side Speed = Forward + Turn
Right Side Speed = Forward - Turn
```

### Priority-Based Intake Control
Prevents conflicting commands when multiple buttons are pressed:
1. **L2 (Highest)** - Both intakes forward
2. **L1** - Both intakes reverse
3. **D-Pad UP** - Blue intake forward only
4. **D-Pad DOWN** - Blue intake reverse only

### Smart Pneumatic Toggles
Edge detection ensures clean on/off behavior:
- One button press = one state change
- No rapid toggling when held
- Independent control for each pneumatic

### Deadband Filtering
Eliminates motor jitter from controller drift by ignoring small joystick movements near center position.

## Competition Modes

### Autonomous Mode
- Runs automatically for 15 seconds at match start
- Displays "Autonomous Mode" on brain screen

### Driver Control Mode
- Full manual control after autonomous
- Displays "Driver Control" on brain screen
- All control mappings active

## 🛠️ Code Structure

### Core Functions

**`apply_deadband(value)`**
- Filters joystick input to prevent drift
- Returns 0 for values within ±5% of center

**`toggle_pneumatic(piston)`**
- Switches piston state (open ↔ closed)
- Simple on/off toggle behavior

**`control_intake(blue_speed, small_speed)`**
- Independent control of both intake motors
- Supports forward, reverse, and stop for each motor

**`autonomous()`**
- Handles 15-second autonomous period
- Placeholder for autonomous routines

**`user_control()`**
- Main driver control loop
- Processes all controller inputs at 50Hz

## 🏁 Getting Started

1. Upload program to VEX V5 Brain
2. Connect controller
3. Verify all motors and pneumatics are properly connected
4. Program displays "Program Started" and "Ready for Competition" on startup
5. Competition object automatically manages mode switching

## 📝 Notes

- All motors use 18:1 gear ratio for optimal balance of speed and torque
- Intake motors are reversed in configuration for correct operational direction
- Third motor on each drivetrain side is reversed to match rotation
- Edge detection prevents accidental pneumatic triggers
- Control loop timing ensures consistent, responsive operation

## 🔧 Customization

To modify control mappings or add features:
1. Unused buttons are clearly marked in code
2. Adjust `INTAKE_SPEED` constant for different power levels
3. Modify `DEADBAND` for different sensitivity
4. Add new functions following existing patterns

---

**Competition Ready** | **50Hz Control Loop** | **Edge Detection** | **Arcade Drive**
