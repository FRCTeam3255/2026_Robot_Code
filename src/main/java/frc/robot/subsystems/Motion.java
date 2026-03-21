// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstMotion;

@Logged
public class Motion extends SubsystemBase {
  /** Creates a new Motion. */
  public final TalonFX intakePivot = new TalonFX(DeviceIDs.motionIDs.INTAKE_PIVOT_CAN);
  public final TalonFX climber = new TalonFX(DeviceIDs.motionIDs.CLIMBER_CAN);
  public final TalonFX hood = new TalonFX(DeviceIDs.motionIDs.HOOD_CAN);

  MotionMagicExpoVoltage climberMotionRequest = new MotionMagicExpoVoltage(0);
  MotionMagicExpoVoltage hoodMotionRequest = new MotionMagicExpoVoltage(0);
  MotionMagicExpoVoltage intakePivotMotionRequest = new MotionMagicExpoVoltage(0);

  VoltageOut intakePivotVoltageRequest = new VoltageOut(0);
  VoltageOut hoodVoltageRequest = new VoltageOut(0);

  public boolean hasIntakePivotZeroed = false;
  public boolean hasHoodZeroed = false;
  public boolean intakePivotAttemptingZeroing = false;
  public boolean hoodAttemptingZeroing = false;

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

  public void setMotorVoltage(Voltage voltage, TalonFX motor) {
    motor.setControl(hoodVoltageRequest.withOutput(voltage));
  }

  public Angle getMotorAngle(TalonFX motor) {
    if (Robot.isSimulation()) {
      return lastDesiredHoodAngle;
    }
    return motor.getPosition().getValue();
  }

  public boolean isMotorAtPosition(Angle tolerance, TalonFX motor) {
    Angle lowerLim = lastDesiredHoodAngle.minus(tolerance);
    Angle upperLim = lastDesiredHoodAngle.plus(tolerance);

    Angle hoodAngle = getMotorAngle(motor);

    hoodAtPosition = hoodAngle.gte(lowerLim)
        && hoodAngle.lte(upperLim);
    return hoodAtPosition;
  }

  public AngularVelocity getRotorVelocity(TalonFX motor) {
    return motor.getRotorVelocity().getValue();
  }

  public boolean isIntakePivotAtPosition(Angle target, Angle tolerance) {// This one is used for the shooting command
    Angle lowerLim = target.minus(tolerance);
    Angle upperLim = target.plus(tolerance);

    Angle pivotAngle = getMotorAngle(intakePivot);
    return pivotAngle.gte(lowerLim) && pivotAngle.lte(upperLim);
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

  public void resetSensorPosition(Angle zeroedPos, TalonFX motor) {
    motor.setPosition(zeroedPos);
  }

  public boolean isClimberAtPosition(Distance distanceTolerance) {
    Distance lowerLim = lastDesiredClimberPosition.minus(distanceTolerance);
    Distance upperLim = lastDesiredClimberPosition.plus(distanceTolerance);

    Distance climberPosition = getClimberPosition();

    climberAtPosition = climberPosition.gte(lowerLim)
        && climberPosition.lte(upperLim);
    return climberAtPosition;
  }

  public boolean isRotorVelocityZero(TalonFX motor) {
    return getRotorVelocity(motor).isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  /**
   * Helper result for zeroing checks. Contains whether the mechanism has been
   * observed at zero for the configured duration, and the updated zeroing
   * timestamp to persist across calls.
   */
  public static class ZeroingResult {
    public final boolean finished;
    public final edu.wpi.first.units.measure.Time timestamp;

    public ZeroingResult(boolean finished, edu.wpi.first.units.measure.Time timestamp) {
      this.finished = finished;
      this.timestamp = timestamp;
    }
  }

  public ZeroingResult checkZeroing(AngularVelocity rotorVelocity, edu.wpi.first.units.measure.Time zeroingTimestamp) {
    // If the caller already considers the subsystem zeroed, short-circuit.
    // (Caller is responsible for that check; we keep helper focused.)

    // If the current velocity is low enough to be considered as zeroed
    if (rotorVelocity.lt(ConstMotion.ZEROED_VELOCITY)) {
      // And this is the first loop it has happened, begin the timer
      if (zeroingTimestamp.equals(Units.Seconds.zero())) {
        return new ZeroingResult(false, Units.Seconds.of(Timer.getFPGATimestamp()));
      }

      // If this isn't the first loop, return if it has been below the threshold for
      // long enough
      boolean done = Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(ConstMotion.ZEROED_TIME);
      return new ZeroingResult(done, zeroingTimestamp);
    }

    // If the above wasn't true, we have gained too much velocity, so we aren't at 0
    // & need to restart the timer
    return new ZeroingResult(false, Units.Seconds.zero());
  }
}
