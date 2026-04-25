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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class ConstMotion {
  public static final TalonFXConfiguration INTAKE_PIVOT_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration CLIMBER_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration HOOD_CONFIGURATION = new TalonFXConfiguration();
  public static final double STOP = 0.0;
  public static final Angle DEPLOY_INTAKE_PIVOT_ANGLE = Units.Degrees.of(125);
  public static final Angle RETRACT_INTAKE_PIVOT_ANGLE = Units.Degrees.of(0);
  public static final Angle LIFT_INTAKE_SHOOTING_ANGLE = Units.Degrees.of(63);
  public static final double EXTEND_CLIMBER_SPEED = 0;
  public static final double RETRACT_CLIMBER_SPEED = -0.5;
  public static final double ANGLE_HOOD_SPEED = 0.5;
  public static final Angle HOOD_TRENCH_ANGLE = Units.Degrees.of(13);
  public static final Angle HOOD_CORNER_ANGLE = Units.Degrees.of(23.5);
  public static final Angle HOOD_NON_OUTPOST_ANGLE = Units.Degrees.of(12);
  public static final Angle HOOD_TOWER_ANGLE = Units.Degrees.of(14);
  public static final Angle HOOD_HUB_ANGLE = Units.Degrees.of(9.5); // TODO: adjust
  public static final Angle HOOD_DEPOT_ANGLE = Units.Degrees.of(18);
  public static final Angle HOOD_ANYWHERE_ANGLE = Units.Degrees.of(45);
  public static final Angle HOOD_NUETRAL_TO_ALLIANCE_ANGLE = Units.Degrees.of(45);
  public static final Angle HOOD_OPPENENT_TO_ALLIANCE_ANGLE = Units.Degrees.of(37);
  public static final Angle HOOD_NONE_ANGLE = Units.Degrees.of(1.6); // Do not change, it's not at zero because the hood
                                                                     // is not perfectly at 0 when the encoder reads 0
  public static final Distance STORE_CLIMBER = Inches.of(0);
  public static final Distance RETRACT_CLIMBER = Inches.of(1.5);
  public static final Distance EXTEND_CLIMBER = Inches.of(13.1);
  public static final Angle HOOD_TOLERANCE = Units.Degrees.of(1.0);
  public final static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  public static final Time LIFT_INTAKE_INTERVAL_TIME = Units.Seconds.of(1.0 / 3.0);
  public static final Angle INTAKE_PIVOT_ANGLE_TOLERANCE = Units.Degrees.of(5);
  public static final int CLIMBER_SLOW_PID_SLOT = 0;
  public static final int CLIMBER_FAST_PID_SLOT = 1;

  public static final Distance CLIMBER_TOLERANCE = Inches.of(0.2);
  static {

    INTAKE_PIVOT_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    INTAKE_PIVOT_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    INTAKE_PIVOT_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    INTAKE_PIVOT_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    INTAKE_PIVOT_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DEPLOY_INTAKE_PIVOT_ANGLE.in(Rotations);
    INTAKE_PIVOT_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RETRACT_INTAKE_PIVOT_ANGLE.in(Rotations);
    INTAKE_PIVOT_CONFIGURATION.CurrentLimits.StatorCurrentLimitEnable = true;
    INTAKE_PIVOT_CONFIGURATION.CurrentLimits.StatorCurrentLimit = 60;
    INTAKE_PIVOT_CONFIGURATION.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    INTAKE_PIVOT_CONFIGURATION.Slot0.kS = 0.3;
    INTAKE_PIVOT_CONFIGURATION.Slot0.kP = 40;
    INTAKE_PIVOT_CONFIGURATION.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    INTAKE_PIVOT_CONFIGURATION.Feedback.SensorToMechanismRatio = 1.0 / ((16.0 / 42.0) * (18.0 / 60.0) * (12.0 / 60.0));

    CLIMBER_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    CLIMBER_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CLIMBER_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    CLIMBER_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    CLIMBER_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitThreshold = EXTEND_CLIMBER.in(Inches);
    CLIMBER_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitThreshold = STORE_CLIMBER.in(Inches);
    // Slot 0
    CLIMBER_CONFIGURATION.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    CLIMBER_CONFIGURATION.Slot0.kS = 0.15;
    CLIMBER_CONFIGURATION.Slot0.kP = 1;
    CLIMBER_CONFIGURATION.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    // Slot 1
    CLIMBER_CONFIGURATION.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    CLIMBER_CONFIGURATION.Slot1.kS = 0.15;
    CLIMBER_CONFIGURATION.Slot1.kP = 20;
    CLIMBER_CONFIGURATION.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    CLIMBER_CONFIGURATION.Feedback.SensorToMechanismRatio = 3.0769;

    HOOD_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    HOOD_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    HOOD_CONFIGURATION.Feedback.SensorToMechanismRatio = 1.0 / ((10.0 / 42.0) * (10.0 / 210.0));
    HOOD_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    HOOD_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    HOOD_CONFIGURATION.CurrentLimits.StatorCurrentLimitEnable = true;
    HOOD_CONFIGURATION.CurrentLimits.StatorCurrentLimit = 60;

    HOOD_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Degrees.of(46).in(Rotations);
    // Do not change, it's not at zero because the hood is not perfectly at 0 when
    // the encoder reads 0
    HOOD_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Degrees.of(1.6).in(Rotations);
    HOOD_CONFIGURATION.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    HOOD_CONFIGURATION.Slot0.kS = 0.2;
    HOOD_CONFIGURATION.Slot0.kP = 300;
    HOOD_CONFIGURATION.Slot0.kG = 0.1;
    HOOD_CONFIGURATION.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    // hoodAngleMap.put(Inches.of(0).in(Inches), Degrees.of(3).in(Degrees));
    hoodAngleMap.put(Inches.of(200).in(Inches), Degrees.of(29).in(Degrees));
    // hoodAngleMap.put(Inches.of(190).in(Inches), Degrees.of(29).in(Degrees));
    hoodAngleMap.put(Inches.of(180).in(Inches), Degrees.of(24).in(Degrees));
    // hoodAngleMap.put(Inches.of(170).in(Inches), Degrees.of(24).in(Degrees));
    // hoodAngleMap.put(Inches.of(160).in(Inches), Degrees.of(24).in(Degrees));
    // hoodAngleMap.put(Inches.of(150).in(Inches), Degrees.of(24).in(Degrees));
    hoodAngleMap.put(Inches.of(140).in(Inches), Degrees.of(14).in(Degrees)); // good
    // hoodAngleMap.put(Inches.of(130).in(Inches), Degrees.of(16).in(Degrees));
    hoodAngleMap.put(Inches.of(120).in(Inches), Degrees.of(13).in(Degrees)); // good
    // hoodAngleMap.put(Inches.of(110).in(Inches), Degrees.of(13).in(Degrees));
    hoodAngleMap.put(Inches.of(100).in(Inches), Degrees.of(8).in(Degrees)); // little too far, made most
    // hoodAngleMap.put(Inches.of(90).in(Inches), Degrees.of(9).in(Degrees));
    hoodAngleMap.put(Inches.of(80).in(Inches), Degrees.of(5.25).in(Degrees)); //
    // hoodAngleMap.put(Inches.of(70).in(Inches), Degrees.of(6).in(Degrees));
    // hoodAngleMap.put(Inches.of(60).in(Inches), Degrees.of(6).in(Degrees));
    hoodAngleMap.put(Inches.of(50).in(Inches), Degrees.of(1).in(Degrees)); //
  }
}