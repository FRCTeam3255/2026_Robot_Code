// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
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
  // Assume scheduler runs ~50Hz; use a fixed loop period instead of Timer.
  private static final double LOOP_PERIOD = 0.02;

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
    // initialize accumulated heading from current robot heading
    targetHeading = subDrivetrain.getDriveRotation();
  }

  @Override
  public void execute() {
    double rotInput = rotationXAxis.getAsDouble();
    double deadband = 0.05;

    if (Math.abs(rotInput) > deadband) {
      double radPerSec = ConstDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond);
      double deltaRad = rotInput * radPerSec * LOOP_PERIOD;
      targetHeading = new Rotation2d(targetHeading.getRadians() + deltaRad);
      subDrivetrain.setDriveRotation(targetHeading.getMeasure());
      isDriverRotationManualInput = true;
    } else {
      isDriverRotationManualInput = false;
      subDrivetrain.setDriveRotation(targetHeading.getMeasure());
    }
    ChassisSpeeds velocities = subDrivetrain.calculateVelocitiesFromInput(
        xAxis,
        yAxis,
        () -> 0.0,
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

    System.out.printf("reqDeg=%.2f reqRad=%.3f actualYawDeg=%.2f actualYawRad=%.3f%n",
        targetHeading.getDegrees(), targetHeading.getRadians(),
        subDrivetrain.getPose().getRotation().getDegrees(), subDrivetrain.getPose().getRotation().getRadians());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
