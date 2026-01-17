// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstPoseDrive.PoseDriveGroup;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseDrive extends Command {
  /** Creates a new PoseDrive. */
  Drivetrain subDrivetrain;
  DriverStateMachine subDriverStateMachine;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  BooleanSupplier slowMode;
  PoseDriveGroup poseGroup;
  Pose2d closestPose;
  public boolean isPoseAligned = false;

  public PoseDrive(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
      DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis, BooleanSupplier slowMode,
      PoseDriveGroup poseGroup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.poseGroup = poseGroup;
    this.slowMode = slowMode;
    addRequirements(this.subDriverStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    closestPose = subDrivetrain.getPose().nearest(poseGroup.targetPoseGroup);
    subDrivetrain.lastDesiredPoseGroup = poseGroup;
    subDrivetrain.lastDesiredTarget = closestPose;

    ChassisSpeeds velocities = subDrivetrain.calculateVelocitiesFromInput(
        xAxis,
        yAxis,
        rotationAxis,
        slowMode,
        ConstField.isRedAlliance(),
        ConstDrivetrain.SLOW_MODE_MULTIPLIER,
        ConstDrivetrain.REAL_DRIVE_SPEED,
        ConstDrivetrain.TURN_SPEED);

    boolean isInAutoDriveZone = subDrivetrain.isInAutoDriveZone(
        poseGroup.minDistanceBeforeDrive,
        closestPose);

    if (isInAutoDriveZone) {
      subDrivetrain.autoAlign(
          closestPose,
          velocities,
          poseGroup.lockX,
          poseGroup.lockY);
      subDriverStateMachine.setDriverState(poseGroup.driveState);
    } else {
      subDrivetrain.rotationalAlign(
          closestPose,
          velocities);
      subDriverStateMachine.setDriverState(poseGroup.snapState);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (closestPose == null) {
      return false;
    }
    isPoseAligned = subDrivetrain.isAtPosition(closestPose, poseGroup.distanceTolerance) &&
        subDrivetrain.isAtRotation(closestPose.getRotation(), poseGroup.rotationTolerance);
    return isPoseAligned;
  }
}
