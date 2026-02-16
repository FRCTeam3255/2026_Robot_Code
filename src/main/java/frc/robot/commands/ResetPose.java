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
    addRequirements(RobotContainer.driverStateMachineInstance);
  }

  @Override
  public void initialize() {
    RobotContainer.drivetrainInstance.resetPose(getAlliancePose());
    RobotContainer.drivetrainInstance.getPigeon2()
        .setYaw(getAlliancePose().getRotation().getMeasure());
    RobotContainer.drivetrainInstance.setDriveRotation(getAlliancePose().getRotation().getMeasure());
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  public Pose2d getAlliancePose() {
    return ConstField.FieldElementGroups.RESET_POSE_SET.getAlliancePoses().get(0);
  }
}
