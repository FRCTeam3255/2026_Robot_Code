// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstField;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;

public class DriveManual extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationXAxis, rotationYAxis;
  boolean isOpenLoop;
  DriverStateMachine subDriverStateMachine;
  BooleanSupplier slowMode;
  Rotation2d targetHeading;
  public boolean isDriverRotationManualInput;
  Timer delayTimer = new Timer();

  public DriveManual(Drivetrain subDrivetrain, DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationXAxis, DoubleSupplier rotationYAxis, DriverStateMachine subDriverStateMachine,
      BooleanSupplier slowMode) {
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationXAxis = rotationXAxis;
    this.rotationYAxis = rotationYAxis;
    this.slowMode = slowMode;
    isOpenLoop = true;

    addRequirements(RobotContainer.subDriverStateMachine);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ChassisSpeeds velocities = subDrivetrain.calculateVelocitiesFromInput(
        xAxis,
        yAxis,
        rotationXAxis,
        slowMode,
        ConstField.isRedAlliance(),
        ConstDrivetrain.SLOW_MODE_MULTIPLIER,
        ConstDrivetrain.REAL_DRIVE_SPEED,
        ConstDrivetrain.TURN_SPEED);

    subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.MANUAL);

    double rotInput = -rotationXAxis.getAsDouble();

    if (Math.abs(rotInput) > ConstDrivetrain.ROTATION_STICK_DEADBAND) {
      subDrivetrain.drive(velocities);
      subDrivetrain.setDriveRotation(subDrivetrain.getPose().getRotation().getMeasure());
      delayTimer.reset();
    } else {
      delayTimer.start();
      if (delayTimer.hasElapsed(ConstDrivetrain.ROTATION_DELAY)) {
        subDrivetrain.drive(
            velocities,
            subDrivetrain.getDriveRotation(),
            ConstDrivetrain.ROTATION_PID.kP,
            ConstDrivetrain.ROTATION_PID.kI,
            ConstDrivetrain.ROTATION_PID.kD);
      } else {
        subDrivetrain.drive(velocities);
        subDrivetrain.setDriveRotation(subDrivetrain.getPose().getRotation().getMeasure());

      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    delayTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
