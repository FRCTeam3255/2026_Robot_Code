// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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

  VoltageOut voltageRequest = new VoltageOut(0);

  public boolean hasIntakePivotZeroed = false;
  public boolean hasHoodZeroed = false;
  public boolean intakePivotAttemptingZeroing = false;
  public boolean hoodAttemptingZeroing = false;

  Angle lastDesiredHoodAngle = Degrees.zero();
  Angle lastDesiredIntakePivotAngle = Degrees.zero();
  Distance lastDesiredClimberPosition = Inches.zero();

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
    lastDesiredIntakePivotAngle = setPoint;
  }

  public void setHoodAngle(Angle setPoint) {
    hood.setControl(hoodMotionRequest.withPosition(setPoint));
    lastDesiredHoodAngle = setPoint;
  }

  public void setClimberPosition(Distance setpoint) {
    climber.setControl(climberMotionRequest.withPosition(setpoint.in(Units.Inches)));
    lastDesiredClimberPosition = setpoint;
  }

  public void setIntakePivotSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    ConstMotion.INTAKE_PIVOT_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
    ConstMotion.INTAKE_PIVOT_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;
    intakePivot.getConfigurator().apply(ConstMotion.INTAKE_PIVOT_CONFIGURATION);
  }

  public void setHoodSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    ConstMotion.HOOD_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
    ConstMotion.HOOD_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;
    hood.getConfigurator().apply(ConstMotion.HOOD_CONFIGURATION);
  }

  public void setIntakePivotVoltage(Voltage voltage) {
    intakePivot.setControl(voltageRequest.withOutput(voltage));
  }

  public void setHoodVoltage(Voltage voltage) {
    hood.setControl(voltageRequest.withOutput(voltage));
  }

  public Angle getPivotAngle() {
    if (Robot.isSimulation()) {
      return lastDesiredIntakePivotAngle;
    }
    return intakePivot.getPosition().getValue();
  }

  public Angle getHoodAngle() {
    if (Robot.isSimulation()) {
      return lastDesiredHoodAngle;
    }
    return hood.getPosition().getValue();
  }

  public boolean isHoodAtPosition(Angle tolerance) {
    Angle lowerLim = lastDesiredHoodAngle.minus(tolerance);
    Angle upperLim = lastDesiredHoodAngle.plus(tolerance);

    Angle hoodAngle = getHoodAngle();

    return hoodAngle.gte(lowerLim)
        && hoodAngle.lte(upperLim);
  }

  public boolean isIntakePivotAtPosition(Angle tolerance) {
    Angle lowerLim = lastDesiredIntakePivotAngle.minus(tolerance);
    Angle upperLim = lastDesiredIntakePivotAngle.plus(tolerance);

    Angle pivotAngle = getPivotAngle();

    return pivotAngle.gte(lowerLim)
        && pivotAngle.lte(upperLim);
  }

  public AngularVelocity getIntakePivotRotorVelocity() {
    return intakePivot.getRotorVelocity().getValue();
  }

  public AngularVelocity getHoodRotorVelocity() {
    return hood.getRotorVelocity().getValue();
  }

  public static Angle getMappedHoodAngle(Distance distance) {
    return Degrees.of(ConstMotion.hoodAngleMap.get(distance.in(Inches)));
  }

  public Distance getClimberPosition() {
    if (Robot.isSimulation()) {
      return lastDesiredClimberPosition;
    }
    return Units.Inches.of(climber.getPosition().getValueAsDouble());
  }

  public void resetIntakePivotSensorPosition(Angle zeroedPos) {
    intakePivot.setPosition(zeroedPos);
  }

  public void resetHoodSensorPosition(Angle zeroedPos) {
    hood.setPosition(zeroedPos);
  }

  public boolean isClimberAtPosition(Distance distanceTolerance) {
    Distance lowerLim = lastDesiredClimberPosition.minus(distanceTolerance);
    Distance upperLim = lastDesiredClimberPosition.plus(distanceTolerance);

    Distance climberPosition = getClimberPosition();

    return climberPosition.gte(lowerLim)
        && climberPosition.lte(upperLim);
  }

  public boolean isIntakePivotRotorVelocityZero() {
    return getIntakePivotRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public boolean isHoodRotorVelocityZero() {
    return getHoodRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }
}
