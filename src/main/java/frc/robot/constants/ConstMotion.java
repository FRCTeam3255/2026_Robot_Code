// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ConstMotion {
  public static final TalonFXConfiguration INTAKE_PIVOT_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration CLIMBER_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration HOOD_CONFIGURATION = new TalonFXConfiguration();
  public static final double STOP = 0.0;
  public static final Angle DEPLOY_INTAKE_PIVOT_ANGLE = Units.Degrees.of(135); // TODO: set angle for intake pivot
  public static final Angle RETRACT_INTAKE_PIVOT_ANGLE = Units.Degrees.of(0); // TODO: set angle for intake pivot
  public static final double EXTEND_CLIMBER_SPEED = 0;
  public static final double RETRACT_CLIMBER_SPEED = -0.5;
  public static final double ANGLE_HOOD_SPEED = 0.5;
  public static final Angle HOOD_TRENCH_ANGLE = Units.Degrees.of(18);
  public static final Angle HOOD_OUTPOST_ANGLE = Units.Degrees.of(45);
  public static final Angle HOOD_NON_OUTPOST_ANGLE = Units.Degrees.of(45);
  public static final Angle HOOD_DEPOT_ANGLE = Units.Degrees.of(45);
  public static final Angle HOOD_ANYWHERE_ANGLE = Units.Degrees.of(45);
  public static final Angle HOOD_NUETRAL_TO_ALLIANCE_ANGLE = Units.Degrees.of(45);
  public static final Angle HOOD_OPPENENT_TO_ALLIANCE_ANGLE = Units.Degrees.of(45);
  public static final Angle HOOD_NONE_ANGLE = Units.Degrees.of(1.6);
  public static final Distance RETRACT_CLIMBER = Inches.of(0);
  public static final Distance EXTEND_CLIMBER = Inches.of(0);
  public static final Angle HOOD_TOLERANCE = Units.Degrees.of(.2);
  public final static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  public static final Distance CLIMBER_TOLERANCE = Inches.of(0.2);

  public static final Angle ZEROED_INTAKE_MANUAL_POS = Units.Degrees.of(0);// TODO: update with actual values
  public static final Angle ZEROED_INTAKE_AUTO_POS = Units.Degrees.of(0);// TODO: update with actual values
  public static final Angle ZEROED_HOOD_MANUAL_POS = Units.Degrees.of(0);// TODO: update with actual values
  public static final Angle ZEROED_HOOD_AUTO_POS = Units.Degrees.of(0);// TODO: update with actual values

  public static final Voltage ZEROING_INTAKE_VOLTAGE = Units.Volts.of(2);
  public static final Voltage ZEROING_HOOD_VOLTAGE = Units.Volts.of(2);

  public static final AngularVelocity MANUAL_ZEROING_START_VELOCITY = Units.RotationsPerSecond.of(5);
  public static final AngularVelocity MANUAL_ZEROING_DELTA_VELOCITY = Units.RotationsPerSecond.of(.1);
  public static final AngularVelocity ZEROED_VELOCITY = Units.RotationsPerSecond.of(0.2);

  public static final Time ZEROING_TIMEOUT = Units.Seconds.of(3);
  public static final Time ZEROED_TIME = Units.Seconds.of(1);

  static {

    INTAKE_PIVOT_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    INTAKE_PIVOT_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    INTAKE_PIVOT_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    INTAKE_PIVOT_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    INTAKE_PIVOT_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Degree.of(135).in(Rotations);
    INTAKE_PIVOT_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Degrees.of(0).in(Rotations);
    INTAKE_PIVOT_CONFIGURATION.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    INTAKE_PIVOT_CONFIGURATION.Slot0.kS = 0.3;
    INTAKE_PIVOT_CONFIGURATION.Slot0.kP = 40;
    INTAKE_PIVOT_CONFIGURATION.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    INTAKE_PIVOT_CONFIGURATION.Feedback.SensorToMechanismRatio = 1.0 / ((16.0 / 42.0) * (18.0 / 60.0) * (12.0 / 60.0));

    CLIMBER_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    CLIMBER_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    HOOD_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    HOOD_CONFIGURATION.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    HOOD_CONFIGURATION.Feedback.SensorToMechanismRatio = 1.0 / ((10.0 / 44.0) * (10.0 / 190.0));
    HOOD_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    HOOD_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    HOOD_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Degrees.of(46).in(Rotations);
    HOOD_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Degrees.of(1.6).in(Rotations);
    HOOD_CONFIGURATION.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    HOOD_CONFIGURATION.Slot0.kS = 0.2;
    HOOD_CONFIGURATION.Slot0.kP = 300;
    HOOD_CONFIGURATION.Slot0.kG = 0.1;
    HOOD_CONFIGURATION.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    hoodAngleMap.put(Inches.of(180).in(Inches), Degrees.of(45).in(Degrees));
    hoodAngleMap.put(Inches.of(120).in(Inches), Degrees.of(40).in(Degrees));
    hoodAngleMap.put(Inches.of(100).in(Inches), Degrees.of(35).in(Degrees));
    hoodAngleMap.put(Inches.of(80).in(Inches), Degrees.of(30).in(Degrees));
    hoodAngleMap.put(Inches.of(90).in(Inches), Degrees.of(25).in(Degrees));
    hoodAngleMap.put(Inches.of(50).in(Inches), Degrees.of(0).in(Degrees));
  }
}