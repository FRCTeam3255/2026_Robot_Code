// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstMotion;

@Logged
public class RobotPoses extends SubsystemBase {
  /** Creates a new RobotPoses. */
  public static final Distance HOPPER_ELEVATION = Inches.of(0);

  Pose3d modelDrivetrain = Pose3d.kZero;
  Pose3d model0IntakePivot = Pose3d.kZero;
  Pose3d model1Hopper = Pose3d.kZero;
  Pose3d model2Shooter = Pose3d.kZero;
  Pose3d model3Hood = Pose3d.kZero;
  Pose3d model4ClimberBaseStage = Pose3d.kZero;

  Transform3d climberTransform3d = new Transform3d();
  Rotation3d intakePivotRotation3d = Rotation3d.kZero;
  Transform3d hopperTransform3d = new Transform3d();
  Rotation3d hoodPivotRotation3d = Rotation3d.kZero;

  Transform3d hoodPivotPoint = new Transform3d(
      Units.Inches.of(10.963),
      Units.Inches.zero(),
      Units.Inches.of(18.516),
      Rotation3d.kZero);
  // Keep a cached distance that the logger can read; updated each scheduler cycle
  private Distance distanceToHub = Units.Meters.of(0);

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
    if (RobotContainer.motionInstance.getMotorAngle(RobotContainer.motionInstance.intakePivot)
        .equals(ConstMotion.DEPLOY_INTAKE_PIVOT_ANGLE)) {
      hopperTransform3d = new Transform3d(
          Units.Inches.zero(),
          Units.Inches.zero(),
          HOPPER_ELEVATION,
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
        RobotContainer.motionInstance.getMotorAngle(RobotContainer.motionInstance.intakePivot),
        Units.Degrees.zero());
    hoodPivotRotation3d = new Rotation3d(
        Units.Degrees.zero(),
        RobotContainer.motionInstance.getMotorAngle(RobotContainer.motionInstance.hood),
        Units.Degrees.zero());

    // Component Positions
    modelDrivetrain = new Pose3d(RobotContainer.drivetrainInstance.getPose());
    model0IntakePivot = Pose3d.kZero.rotateAround(
        Pose3d.kZero.plus(intakePivotPoint).getTranslation(), intakePivotRotation3d);
    model1Hopper = Pose3d.kZero.transformBy(hopperTransform3d);
    model2Shooter = Pose3d.kZero;
    model3Hood = Pose3d.kZero
        .rotateAround(Pose3d.kZero.plus(hoodPivotPoint).getTranslation(), hoodPivotRotation3d);

    Pose2d hubPose = getHub();
    distanceToHub = Units.Meters.of(
        RobotContainer.drivetrainInstance.getPose().getTranslation().getDistance(hubPose.getTranslation()));
  }

  public Pose2d getHub() {
    return ConstField.FieldElementGroups.HUB_POSE_SET.getAlliancePoses().get(0);
  }

  public Distance getDistanceToHub() {
    return distanceToHub;
  }
}