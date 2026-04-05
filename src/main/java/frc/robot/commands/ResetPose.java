// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstVision;

public class ResetPose extends Command {

  public ResetPose() {
    addRequirements(RobotContainer.driverStateMachineInstance);
  }

  @Override
  public void initialize() {
    RobotContainer.visionInstance.setVisionEnabled(false);
    RobotContainer.drivetrainInstance.resetPoseAndYaw(getAlliancePose());
  }

  @Override
  public void execute() {
    RobotContainer.visionInstance.setIMUAssistMode(false);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.visionInstance.setIMUAssistMode(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public Pose2d getAlliancePose() {
    return ConstField.FieldElementGroups.RESET_POSE_SET.getAlliancePoses().get(0);
  }
}
