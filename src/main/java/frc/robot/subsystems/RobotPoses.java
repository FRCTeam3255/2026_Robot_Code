// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstMotion;

@Logged
public class RobotPoses extends SubsystemBase {
  /** Creates a new RobotPoses. */

  Pose3d modelDrivetrain = Pose3d.kZero;
  Pose3d model0IntakePivot = Pose3d.kZero;
  Pose3d model1Hopper = Pose3d.kZero;
  Pose3d model2Shooter = Pose3d.kZero;
  Pose3d model3Hood = Pose3d.kZero;
  Pose3d model4ClimberBaseStage = Pose3d.kZero;

  Transform3d climberTransform3d;
  Rotation3d intakePivotRotation3d;
  Transform3d hopperTransform3d;
  Rotation3d hoodPivotRotation3d;

  Transform3d hoodPivotPoint = new Transform3d(
      Units.Inches.zero(),
      Units.Inches.of(18.516),
      Units.Inches.of(10.963),
      Rotation3d.kZero);

  Transform3d intakePivotPoint = new Transform3d(
      Units.Inches.zero(),
      Units.Inches.of(8.193),
      Units.Inches.of(7.731),
      Rotation3d.kZero);

  public RobotPoses() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.motionInstance.getPivotAngle() == ConstMotion.DEPLOY_INTAKE_PIVOT_ANGLE) {
      hopperTransform3d = new Transform3d(
          Units.Inches.zero(),
          Units.Inches.zero(),
          ConstMotion.HOPPER_ELEVATION,
          Rotation3d.kZero);
    } else {
      hopperTransform3d = new Transform3d(
          Units.Inches.zero(),
          Units.Inches.zero(),
          Units.Inches.zero(),
          Rotation3d.kZero);
    }
    climberTransform3d = new Transform3d(
        Units.Inches.zero(),
        RobotContainer.motionInstance.getClimberPosition().div(2),
        Units.Inches.zero(),
        Rotation3d.kZero);
    intakePivotRotation3d = new Rotation3d(
        Units.Degrees.zero(),
        RobotContainer.motionInstance.getPivotAngle(),
        Units.Degrees.zero());
    hoodPivotRotation3d = new Rotation3d(
        Units.Degrees.zero(),
        RobotContainer.motionInstance.getHoodAngle().unaryMinus(),
        Units.Degrees.zero());

    // Component Positions
    modelDrivetrain = new Pose3d(RobotContainer.drivetrainInstance.getPose());
    model0IntakePivot = Pose3d.kZero.rotateAround(
        Pose3d.kZero.plus(intakePivotPoint).getTranslation(), intakePivotRotation3d);
    model1Hopper = Pose3d.kZero.transformBy(hopperTransform3d);
    model2Shooter = Pose3d.kZero;
    model3Hood = Pose3d.kZero
        .rotateAround(Pose3d.kZero.plus(hoodPivotPoint).getTranslation(), hoodPivotRotation3d);
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