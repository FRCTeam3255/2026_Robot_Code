// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstField;
import frc.robot.subsystems.DriverStateMachine.DriverState;

public class DriveManual extends Command {
  DoubleSupplier xAxis, yAxis, rotationXAxis, rotationYAxis;
  boolean isOpenLoop;
  BooleanSupplier slowMode;
  public boolean isDriverRotationManualInput;
  Timer delayTimer = new Timer();

  public DriveManual(DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationXAxis, DoubleSupplier rotationYAxis,
      BooleanSupplier slowMode) {
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationXAxis = rotationXAxis;
    this.rotationYAxis = rotationYAxis;
    this.slowMode = slowMode;
    isOpenLoop = true;

    addRequirements(RobotContainer.driverStateMachineInstance);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ChassisSpeeds velocities = RobotContainer.drivetrainInstance.calculateVelocitiesFromInput(
        xAxis,
        yAxis,
        rotationXAxis,
        slowMode,
        ConstField.isRedAlliance(),
        ConstDrivetrain.SLOW_MODE_MULTIPLIER,
        ConstDrivetrain.REAL_DRIVE_SPEED,
        ConstDrivetrain.TURN_SPEED);

    RobotContainer.driverStateMachineInstance.setDriverState(DriverState.MANUAL);

    double rotInput = -rotationXAxis.getAsDouble();

    if (Math.abs(rotInput) > ConstDrivetrain.ROTATION_STICK_DEADBAND) {
      RobotContainer.drivetrainInstance.setIsManualRotationEnabled(true);
      RobotContainer.drivetrainInstance.drive(velocities);
      RobotContainer.drivetrainInstance
          .setDriveRotation(RobotContainer.drivetrainInstance.getPose().getRotation().getMeasure());
      delayTimer.reset();
    } else {
      delayTimer.start();
      if (delayTimer.hasElapsed(ConstDrivetrain.ROTATION_DELAY)) {
        RobotContainer.drivetrainInstance.drive(
            velocities,
            RobotContainer.drivetrainInstance.getTargetRotation(),
            ConstDrivetrain.ROTATION_PID.kP,
            ConstDrivetrain.ROTATION_PID.kI,
            ConstDrivetrain.ROTATION_PID.kD);
      } else {
        RobotContainer.drivetrainInstance.drive(velocities);
        RobotContainer.drivetrainInstance
            .setDriveRotation(RobotContainer.drivetrainInstance.getPose().getRotation().getMeasure());

      }
      double rightStickX = xAxis.getAsDouble();
      double rightStickY = yAxis.getAsDouble();

      if (Math.abs(rightStickX) > ConstDrivetrain.ROTATION_STICK_DEADBAND || Math.abs(rightStickY) > ConstDrivetrain.ROTATION_STICK_DEADBAND) {
        

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
