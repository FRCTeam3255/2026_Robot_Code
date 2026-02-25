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
    RobotContainer.drivetrainInstance.resetPose(getAlliancePose());
    RobotContainer.drivetrainInstance.getPigeon2()
        .setYaw(getAlliancePose().getRotation().getMeasure());
    RobotContainer.drivetrainInstance.setDriveRotation(getAlliancePose().getRotation().getMeasure());
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.DisabledThrottle);
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.DisabledThrottle);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, 1);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, 1);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.TeleopThrottle);
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.TeleopThrottle);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, 4);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, 4);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  public Pose2d getAlliancePose() {
    return ConstField.FieldElementGroups.RESET_POSE_SET.getAlliancePoses().get(0);
  }
}
