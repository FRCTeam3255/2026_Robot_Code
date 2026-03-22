// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs;
import frc.robot.Robot;
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

  Angle lastDesiredHoodAngle = Degrees.zero();
  Angle lastDesiredIntakePivotAngle = Degrees.zero();
  Distance lastDesiredClimberPosition = Inches.zero();

  private boolean hoodAtPosition = false;
  private boolean intakePivotAtPosition = false;
  private boolean climberAtPosition = false;

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

  public void setClimberPosition(Distance setpoint, int slot) {
    climber.setControl(climberMotionRequest.withPosition(setpoint.in(Units.Inches)).withSlot(slot));
    lastDesiredClimberPosition = setpoint;
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

    hoodAtPosition = hoodAngle.gte(lowerLim)
        && hoodAngle.lte(upperLim);
    return hoodAtPosition;
  }

  public boolean isIntakePivotAtPosition(Angle tolerance) {
    Angle lowerLim = lastDesiredIntakePivotAngle.minus(tolerance);
    Angle upperLim = lastDesiredIntakePivotAngle.plus(tolerance);

    Angle pivotAngle = getPivotAngle();

    intakePivotAtPosition = pivotAngle.gte(lowerLim)
        && pivotAngle.lte(upperLim);
    return intakePivotAtPosition;
  }

  public boolean isIntakePivotAtPosition(Angle target, Angle tolerance) {
    Angle lowerLim = target.minus(tolerance);
    Angle upperLim = target.plus(tolerance);

    Angle pivotAngle = getPivotAngle();

    intakePivotAtPosition = pivotAngle.gte(lowerLim)
        && pivotAngle.lte(upperLim);
    return intakePivotAtPosition;
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

  public boolean isClimberAtPosition(Distance distanceTolerance) {
    Distance lowerLim = lastDesiredClimberPosition.minus(distanceTolerance);
    Distance upperLim = lastDesiredClimberPosition.plus(distanceTolerance);

    Distance climberPosition = getClimberPosition();

    climberAtPosition = climberPosition.gte(lowerLim)
        && climberPosition.lte(upperLim);
    return climberAtPosition;
  }
}
