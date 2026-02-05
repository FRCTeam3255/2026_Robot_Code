// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstSystem;

@Logged
public class RobotPoses extends SubsystemBase {
  /** Creates a new RobotPoses. */

  Pose3d modelDrivetrain = Pose3d.kZero;
  Pose3d model0IntakePivot = Pose3d.kZero;
  Pose3d model1Hopper = Pose3d.kZero;
  Pose3d model2Shooter = Pose3d.kZero;
  Pose3d model3ClimberBaseStage = Pose3d.kZero;

  Transform3d climberTransform3d;
  Rotation3d intakePivotRotation3d;
  Transform3d hopperTransform3d;
  Rotation3d hoodPivotRotation3d;

  Transform3d intakePivotPoint = new Transform3d(
      Units.Inches.zero(),
      Units.Inches.zero(),
      Units.Inches.zero(),
      Rotation3d.kZero);

  Transform3d hoodPivotPoint = new Transform3d(
      Units.Inches.zero(),
      Units.Inches.zero(),
      Units.Inches.zero(),
      Rotation3d.kZero);

  public RobotPoses() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climberTransform3d = new Transform3d(
        Units.Inches.zero(),
        RobotContainer.motionInstance.getClimberPosition().div(2),
        Units.Inches.zero(),
        Rotation3d.kZero);
    intakePivotRotation3d = new Rotation3d(
        RobotContainer.motionInstance.getPivotAngle(),
        Units.Degrees.zero(),
        Units.Degrees.zero());
    hopperTransform3d = new Transform3d(
        Units.Inches.zero(),
        Units.Inches.zero(),
        Units.Inches.zero(),
        Rotation3d.kZero);
    hoodPivotRotation3d = new Rotation3d(
        Units.Degrees.zero(),
        RobotContainer.motionInstance.getHoodAngle(),
        Units.Degrees.zero());

    // Robot Positions
    modelDrivetrain = new Pose3d(RobotContainer.drivetrainInstance.getPose());
    model0IntakePivot = Pose3d.kZero.rotateAround(
        Pose3d.kZero.plus(intakePivotPoint).getTranslation(), intakePivotRotation3d);

    model1Hopper = model0IntakePivot.transformBy(hopperTransform3d);

    model2Shooter = Pose3d.kZero;

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

// Pose3d modelDrivetrain = Pose3d.kZero;
// Pose3d model0Pivot = Pose3d.kZero;
// Pose3d model1ElevatorStage2 = Pose3d.kZero;
// Pose3d model2ElevatorCarriage = Pose3d.kZero;
// Pose3d model3Intake = Pose3d.kZero;
// Pose3d model4Climber = Pose3d.kZero;
// Pose3d coralPose = constField.SCORING_ELEMENT_NOT_COLLECTED;
// Pose3d algaePose = constField.SCORING_ELEMENT_NOT_COLLECTED;

// Transform3d elevatorTransform3d;
// Rotation3d pivotRotation3d;
// Rotation3d wristRotation3d;

// // Pivot Point Locations
// Transform3d wristPivotPoint = new Transform3d(
// Units.Inches.zero(),
// Units.Inches.of(16),
// Units.Inches.of(8),
// Rotation3d.kZero);
// Transform3d elevatorPivotPoint = new Transform3d(
// Units.Inches.zero(),
// Units.Inches.of(-9.8),
// Units.Inches.of(8),
// Rotation3d.kZero);

// public RobotPoses(Drivetrain subDrivetrain, Motion subMotion, Rotors
// subRotors) {
// this.subDrivetrain = subDrivetrain;
// this.subMotion = subMotion;
// this.subRotors = subRotors;
// }

// @Override
// public void periodic() {
// // This method will be called once per scheduler run
// elevatorTransform3d = new Transform3d(
// Units.Inches.zero(),
// subMotion.getLiftPosition().div(2),
// Units.Inches.zero(),
// Rotation3d.kZero);
// pivotRotation3d = new Rotation3d(
// subMotion.getPivotAngle(),
// Units.Degrees.zero(),
// Units.Degrees.zero());
// wristRotation3d = new Rotation3d(
// Units.Degrees.zero(),
// subMotion.getWristAngle(),
// Units.Degrees.zero());

// // Robot Positions
// modelDrivetrain = new Pose3d(subDrivetrain.getPose());

// model0Pivot = Pose3d.kZero.rotateAround(
// Pose3d.kZero.plus(elevatorPivotPoint).getTranslation(), pivotRotation3d)
// .rotateBy(new Rotation3d(
// Units.Degrees.zero(),
// Units.Degrees.zero(),
// Units.Degrees.of(-90)));

// model1ElevatorStage2 = model0Pivot.transformBy(elevatorTransform3d);

// model2ElevatorCarriage =
// model1ElevatorStage2.transformBy(elevatorTransform3d);

// // Rotate intake around the end of the elevator carriage (adjust Z offset as
// // needed)
// model3Intake = model2ElevatorCarriage.rotateAround(
// model2ElevatorCarriage.plus(wristPivotPoint).getTranslation(),
// wristRotation3d);

// model4Climber = model0Pivot;
// }
// }
