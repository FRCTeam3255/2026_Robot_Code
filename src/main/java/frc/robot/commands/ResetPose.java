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
  }

  @Override
  public void execute() {
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.IMUMode.EXTERNAL_SEED);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.IMUMode.EXTERNAL_SEED);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_TOP_NAME, ConstVision.IMUMode.EXTERNAL_SEED);
  }

  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.IMUMode.EXTERNAL_ONLY);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.IMUMode.EXTERNAL_ONLY);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_TOP_NAME, ConstVision.IMUMode.EXTERNAL_ONLY);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public Pose2d getAlliancePose() {
    return ConstField.FieldElementGroups.RESET_POSE_SET.getAlliancePoses().get(0);
  }
}
