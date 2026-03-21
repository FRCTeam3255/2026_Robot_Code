// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConstVision;

public class AddVisionMeasurement extends Command {

  Optional<PoseEstimate> estimatedPose;

  public AddVisionMeasurement() {

    addRequirements(RobotContainer.visionInstance);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Tells the limelight where we are on the field
    double yaw = RobotContainer.drivetrainInstance.getPose().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation(ConstVision.LIMELIGHT_RIGHT_NAME,
        yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(ConstVision.LIMELIGHT_LEFT_NAME,
        yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(ConstVision.LIMELIGHT_TOP_NAME,
        yaw, 0, 0, 0, 0, 0);
    AngularVelocity gyroRate = RobotContainer.drivetrainInstance.getGyroRate();
    estimatedPose = RobotContainer.visionInstance.determinePoseEstimate(gyroRate);
    if (estimatedPose.isPresent() && RobotContainer.visionInstance.isVisionEnabled()) {
      RobotContainer.drivetrainInstance.addVisionMeasurement(estimatedPose.get().pose,
          estimatedPose.get().timestampSeconds);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
