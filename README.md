# 2026_Robot_Code — FRC Team 3255

Originally created to host FRC 3255's swerve-drive classes from [SuperCORE](https://github.com/FRCTeam3255/SuperCORE), this repository now contains the example implementation of our SN_SuperSwerve class and the full 2026 robot code.

Jump into the code: `src/main/java/frc/robot`

## Quick links

- Intro to Swerve (presentation): https://docs.google.com/presentation/d/1TPgX1LmMHTN7iIq1K4172TEAYrTftHnFnPqoAa06ExA/edit?usp=sharing
- References:
  - https://github.com/FRCTeam3255/Robot2023
  - https://github.com/dirtbikerxz/BaseTalonFXSwerve
  - https://github.com/SeanSun6814/FRC0ToAutonomous and his [video](https://www.youtube.com/watch?v=0Xi9yb1IMyA)

## Competitions

- FIRST Championship

## Social

- Instagram: https://www.instagram.com/frcteam3255/
- YouTube (Primary): https://www.youtube.com/@FRC3255
- YouTube (Clips): https://www.youtube.com/@FRC3255Clips
- Build thread (Chief Delphi):
- Website: https://supernurds.com

## REBUILT™ presented by Haas

- Game animation: https://www.youtube.com/watch?v=_fybREErgyM

## Software features

### Logging

We use Epilogue as our primary logging framework to record and visualize robot behavior in real time and after matches. We log:

- Motor outputs and velocities
- Current draw
- Sensor values (encoders, vision, etc.)
- Robot pose and estimator data
- State machine states and transitions
- Command execution timing

Key advantage

Logging allows us to understand exactly what the robot was "thinking" during a match. We can replay logs with data, graphs, and the field with robot poses to diagnose issues, tune performance, and identify inconsistencies.

### Simulation

We use WPILib simulation extensively during early development to validate robot logic without physical hardware. Simulation allows running full robot code in a virtual environment, including drivetrain movement, subsystem behavior, and autonomous routines.

In simulation we typically test:

- Autonomous routines and trajectory following
- State machine transitions
- Potential collisions

Key advantage

Simulation enables faster iteration and reduces risk before deploying to the real robot.

### Vision

#### Pose Align — vision-aided alignment

What it does

Pose Align automatically rotates the robot to face the target using vision. "Prep Anywhere" (smart shooting) allows accurate shots from anywhere on the field instead of using fixed presets.

How it works

- Using the robot's current pose we calculate shooting speed and hood angle based on distance using interpolation of calibration data.
- The robot "snaps" to the optimal heading by computing the target angle and applying a PID controller for smooth, precise rotation.
- During alignment the driver retains translational control while rotation is corrected automatically for consistent aiming.

#### Resistance to defence while shooting

We experienced heavy defence at our Division Championship, so the robot dynamically adjusts while shooting.

How it works

- When at least one Limelight detects an AprilTag, the robot enters a generalized aiming state.
- In this state the hood angle, flywheel velocity, and drivetrain heading are continuously updated from vision data.
- If the drivetrain isn't at its target heading, both drivetrain rotation and hood angle are adjusted to converge on the correct shooting configuration.
- Once the robot reaches the desired heading the drivetrain may lock in an X-brake configuration to maintain stability while shooting.

### State machine control

What it is

Our state machine ensures the robot only transitions when it is fully ready, preventing invalid or unsafe actions.

How it works

- We use an enum-based system with nested switch logic to strictly control allowed transitions.
- The tryState method checks whether a requested transition is valid based on the current state and subsystem readiness, sensor inputs, and mechanism positions.
- If valid, tryState returns the new state and executes the associated command. Each state maps to commands with defined subsystem requirements to avoid conflicts.
- In `RobotContainer` we call tryState wrapped in a deferred proxy so the transition can be re-evaluated continuously and adapt to real-time conditions.

Key advantage

This system blocks invalid transitions, improves reliability under match pressure, simplifies debugging, and provides a structured, scalable framework for managing complex robot behavior.

### PID control & Motion Magic

PID is used for:

- Drivetrain heading control
- Shooter velocity regulation
- Vision alignment

Motion Magic is used for mechanisms that require motion profiling (for example: pivot, elevator). Our shooter hood and pivot use CTRE Motion Magic, which allows tuning of PID plus acceleration and cruise velocity.

### Autonomous (Autos)

Our autonomous routines use the command-based structure and trajectory following. Each auto typically combines:

- Preplanned paths (Choreo)
- Vision-assisted corrections
- State-machine-based actions (intake, shoot, align)

Driving modes

- Basic driving: Choreo-generated trajectories
- Pose Target driving: dynamically adjusts robot pose to enable Prep Anywhere (same smart-shooting logic used in teleop)

Commands

State-machine-based command sequencing ensures smooth, reliable execution. Many autonomous behaviors reuse teleop logic (drivetrain PID, alignment logic, state machine transitions), which reduces bugs and keeps behavior consistent between auto and teleop.

Optimizations

The robot will automatically run Prep Anywhere as soon as it crosses the alliance line to reduce aiming time.