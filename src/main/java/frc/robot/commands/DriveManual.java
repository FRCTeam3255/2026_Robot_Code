// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstField;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;

public class DriveManual extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  DriverStateMachine subDriverStateMachine;
  BooleanSupplier slowMode;

  public DriveManual(Drivetrain subDrivetrain, DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, DriverStateMachine subDriverStateMachine, BooleanSupplier slowMode) {
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowMode = slowMode;

    isOpenLoop = true;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ChassisSpeeds velocities = subDrivetrain.calculateVelocitiesFromInput(
        xAxis,
        yAxis,
        rotationAxis,
        slowMode,
        ConstField.isRedAlliance(),
        ConstDrivetrain.SLOW_MODE_MULTIPLIER,
        ConstDrivetrain.REAL_DRIVE_SPEED,
        ConstDrivetrain.TURN_SPEED);

    subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.MANUAL);

    subDrivetrain.drive(velocities);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
