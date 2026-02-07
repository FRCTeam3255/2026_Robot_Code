// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
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
  private double lastTime;

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

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;

    double rotInput = rotationXAxis.getAsDouble();
    double deadband = 0.05;
    if (Math.abs(rotInput) > deadband) {
      double radPerSec = ConstDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond);
      double deltaRad = rotInput * radPerSec * dt;
      Rotation2d current = subDrivetrain.getDriveRotation();
      targetHeading = new Rotation2d(current.getRadians() + deltaRad);
      subDrivetrain.setDriveRotation(targetHeading.getMeasure());
      isDriverRotationManualInput = true;
    } else {
      isDriverRotationManualInput = false;
      targetHeading = subDrivetrain.getDriveRotation();
    }
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

    subDrivetrain.drive(
        velocities,
        subDrivetrain.getDriveRotation(),
        ConstDrivetrain.ROTATION_PID.kP,
        ConstDrivetrain.ROTATION_PID.kI,
        ConstDrivetrain.ROTATION_PID.kD);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
