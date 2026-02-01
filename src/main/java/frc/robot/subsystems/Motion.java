// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstMotion;

@Logged
public class Motion extends SubsystemBase {
  /** Creates a new Motion. */
  final TalonFX intakePivot = new TalonFX(DeviceIDs.motionIDs.INTAKE_PIVOT_CAN);
  final TalonFX climber = new TalonFX(DeviceIDs.motionIDs.CLIMBER_CAN);
  final TalonFX hood = new TalonFX(DeviceIDs.motionIDs.HOOD_CAN);

  MotionMagicExpoVoltage climberMotionRequest = new MotionMagicExpoVoltage(0);
  MotionMagicExpoVoltage hoodMotionRequest = new MotionMagicExpoVoltage(0);
  MotionMagicExpoVoltage intakePivotMotionRequest = new MotionMagicExpoVoltage(0);

  public Motion() {
    intakePivot.getConfigurator().apply(ConstMotion.INTAKE_PIVOT_CONFIGURATION);
    climber.getConfigurator().apply(ConstMotion.CLIMBER_CONFIGURATION);
    hood.getConfigurator().apply(ConstMotion.HOOD_CONFIGURATION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setIntakePivotAngle(Angle setPoint) {
    intakePivot.setControl(intakePivotMotionRequest.withPosition(setPoint));
  }

  public void setHoodAngle(Angle setPoint) {
    hood.setControl(hoodMotionRequest.withPosition(setPoint));

  }

  public void setClimberPosition(Distance setpoint) {
    climber.setControl(climberMotionRequest.withPosition(setpoint.in(Units.Inches)));
  }

  public static Angle getHoodAngle(Distance distance) {
    return Degrees.of(ConstMotion.hoodAngleMap.get(distance.in(Inches)));
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

  public Angle getGlobalHoodAngle() {
    Angle angle = getHoodAngle(Meters.of(RobotContainer.drivetrainInstance.getPose().getTranslation()
        .getDistance(getHub().getTranslation())));

    return angle;
  }
}
