// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveManual;
import frc.robot.commands.PoseDrive;
import frc.robot.constants.ConstPoseDrive;

@Logged
public class DriverStateMachine extends SubsystemBase {
  /** Creates a new DriverStateMachine. */
  public static DriverState currentDriverState = DriverState.MANUAL;

  @NotLogged
  Drivetrain subDrivetrain;
  @NotLogged
  DriverStateMachine subDriverStateMachine = this;

  public DriverStateMachine(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;
  }

  public DriverState getDriverState() {
    return currentDriverState;
  }

  public void setDriverState(DriverState driverState) {
    currentDriverState = driverState;
  }

  public enum DriverState {
    MANUAL,
    EXAMPLE_POSE_DRIVE,
    EXAMPLE_ROTATION_SNAP,
    CHOREO
  }

  public Supplier<Command> tryState(
      DriverState desiredState,
      DoubleSupplier xAxis,
      DoubleSupplier yAxis,
      DoubleSupplier rotationAxis,
      BooleanSupplier slowMode) {
    switch (desiredState) {
      case MANUAL:
        switch (currentDriverState) {
          case MANUAL:
          case EXAMPLE_POSE_DRIVE:
          case CHOREO:
            return () -> new DriveManual(
                subDrivetrain,
                xAxis,
                yAxis,
                rotationAxis,
                subDriverStateMachine,
                slowMode);
        }
        break;

      case EXAMPLE_POSE_DRIVE:
        switch (currentDriverState) {
          case MANUAL:
          case EXAMPLE_POSE_DRIVE:
            return () -> new PoseDrive(
                subDrivetrain,
                subDriverStateMachine,
                xAxis,
                yAxis,
                rotationAxis,
                slowMode,
                ConstPoseDrive.EXAMPLE_POSE_DRIVE_GROUP);
        }
        break;

      case EXAMPLE_ROTATION_SNAP:
        switch (currentDriverState) {
          case EXAMPLE_POSE_DRIVE:
          case EXAMPLE_ROTATION_SNAP:
          case MANUAL:
            return () -> new PoseDrive(
                subDrivetrain,
                subDriverStateMachine,
                xAxis,
                yAxis,
                rotationAxis,
                slowMode,
                ConstPoseDrive.EXAMPLE_POSE_DRIVE_GROUP);
        }

        break;
    }

    return () -> Commands.print("ITS SO OVER D: Invalid Driver State Provided, Blame Eli. Attempted to go to: "
        + desiredState.toString() + " while at " + currentDriverState.toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
