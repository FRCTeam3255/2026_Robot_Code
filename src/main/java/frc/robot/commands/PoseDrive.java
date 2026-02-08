// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstPoseDrive.PoseDriveGroup;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseDrive extends Command {
  /** Creates a new PoseDrive. */

  DoubleSupplier xAxis, yAxis, rotationAxis;
  BooleanSupplier slowMode;
  PoseDriveGroup poseGroup;
  Pose2d closestPose;
  public boolean isPoseAligned = false;

  public PoseDrive(
      DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis, BooleanSupplier slowMode,
      PoseDriveGroup poseGroup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.poseGroup = poseGroup;
    this.slowMode = slowMode;
    addRequirements(RobotContainer.subDriverStateMachineInstance);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    closestPose = RobotContainer.drivetrainInstance.getPose().nearest(poseGroup.targetPoseGroup);
    RobotContainer.drivetrainInstance.lastDesiredPoseGroup = poseGroup;
    RobotContainer.drivetrainInstance.lastDesiredTarget = closestPose;

    ChassisSpeeds velocities = RobotContainer.drivetrainInstance.calculateVelocitiesFromInput(
        xAxis,
        yAxis,
        rotationAxis,
        slowMode,
        ConstField.isRedAlliance(),
        ConstDrivetrain.SLOW_MODE_MULTIPLIER,
        ConstDrivetrain.REAL_DRIVE_SPEED,
        ConstDrivetrain.TURN_SPEED);

    boolean isInAutoDriveZone = RobotContainer.drivetrainInstance.isInAutoDriveZone(
        poseGroup.minDistanceBeforeDrive,
        closestPose);

    if (isInAutoDriveZone) {
      RobotContainer.drivetrainInstance.autoAlign(
          closestPose,
          velocities,
          poseGroup.lockX,
          poseGroup.lockY);
      RobotContainer.subDriverStateMachineInstance.setDriverState(poseGroup.driveState);
    } else {
      RobotContainer.drivetrainInstance.rotationalAlign(
          closestPose,
          velocities);
      RobotContainer.subDriverStateMachineInstance.setDriverState(poseGroup.snapState);
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
    isPoseAligned = RobotContainer.drivetrainInstance.isAtPosition(closestPose, poseGroup.distanceTolerance) &&
        RobotContainer.drivetrainInstance.isAtRotation(closestPose.getRotation(), poseGroup.rotationTolerance);
    return isPoseAligned;
  }
}
