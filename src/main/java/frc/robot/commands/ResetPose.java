// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstField;

public class ResetPose extends Command {

  public ResetPose() {
  }

  @Override
  public void initialize() {
    RobotContainer.drivetrainInstance.resetPose(resetPose());
    RobotContainer.drivetrainInstance.getPigeon2()
        .setYaw(resetPose().getRotation().getMeasure());
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  public Pose2d resetPose() {
    Pose2d resetPose;
    if (ConstField.isRedAlliance()) {
      resetPose = ConstField.FieldElementGroups.RESET_POSE_SET.getRed().get(0);
    } else {
      resetPose = ConstField.FieldElementGroups.RESET_POSE_SET.getBlue().get(0);
    }
    return resetPose;
  }
}
