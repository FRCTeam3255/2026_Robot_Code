// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstSystem;

@Logged
public class RobotPoses extends SubsystemBase {
  /** Creates a new RobotPoses. */

  public Distance distanceToHub = Units.Inches.of(0);
  @NotLogged
  Drivetrain subDrivetrain;

  Pose3d comp0Drivetrain = Pose3d.kZero;
  Pose3d comp1Bumpers = Pose3d.kZero.plus(ConstSystem.ROBOT_TO_BUMPERS);

  public RobotPoses(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Robot Positions
    comp0Drivetrain = new Pose3d(subDrivetrain.getPose());
  }

  public Pose2d getHub() {
    Pose2d hubPose;
    if (ConstField.isRedAlliance()) {
      hubPose = ConstField.FieldElementGroups.HUB_POSE_SET.getRed().get(0);
    } else {
      hubPose = ConstField.FieldElementGroups.HUB_POSE_SET.getBlue().get(0);
    }
    return hubPose;
  }
}
