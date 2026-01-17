// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.frcteam3255.utils.LimelightHelpers;
import com.frcteam3255.utils.LimelightHelpers.PoseEstimate;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConstVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AddVisionMeasurement extends Command {
  Drivetrain subDrivetrain;
  Vision subVision;

  Optional<PoseEstimate> estimatedPose;

  public AddVisionMeasurement(Drivetrain subDrivetrain, Vision subVision) {
    this.subDrivetrain = subDrivetrain;
    this.subVision = subVision;

    addRequirements(subVision);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Tells the limelight where we are on the field
    LimelightHelpers.SetRobotOrientation(ConstVision.LIMELIGHT_RIGHT_NAME,
        subDrivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(ConstVision.LIMELIGHT_LEFT_NAME,
        subDrivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(ConstVision.LIMELIGHT_BACK_NAME,
        subDrivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    AngularVelocity gyroRate = subDrivetrain.getGyroRate();

    estimatedPose = subVision.determinePoseEstimate(gyroRate);
    if (estimatedPose.isPresent()) {
      subDrivetrain.addVisionMeasurement(estimatedPose.get().pose, estimatedPose.get().timestampSeconds);
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
