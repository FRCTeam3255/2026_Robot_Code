// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class ConstRotors {

  public static final TalonFXConfiguration SERIALIZER_ROLLERS_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration INTAKE_ROLLER_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration SHOOTER_TRANSFER_EAST_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration SHOOTER_TRANSFER_WEST_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration FLYWHEEL_WEST_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration FLYWHEEL_EAST_CONFIGURATION = new TalonFXConfiguration();
  public static final double INTAKE_ROLLER_SPEED = 1;
  public static final double SERIALIZER_ROLLERS_SPEED = 1;
  public static final AngularVelocity SHOOTER_TRANSFER_SPEED = RPM.of(3400);
  public static final AngularVelocity FLYWHEEL_TOLERANCE = RPM.of(100);
  public final static InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();

  static {
    // TODO: tune current limits
    SERIALIZER_ROLLERS_CONFIGURATION.CurrentLimits.StatorCurrentLimitEnable = true;
    SERIALIZER_ROLLERS_CONFIGURATION.CurrentLimits.StatorCurrentLimit = 40; // This is just the current limit from the
                                                                            // 2025 hopper
  }

  // different shooting locations shooting speeds
  public static final AngularVelocity FLYWHEEL_TRENCH_SPEED = RPM.of(4000);
  public static final AngularVelocity FLYWHEEL_CORNER_SPEED = RPM.of(4250);
  public static final AngularVelocity FLYWHEEL_NON_OUTPOST_SPEED = RPM.of(4100);
  public static final AngularVelocity FLYWHEEL_TOWER_SPEED = RPM.of(4000);
  public static final AngularVelocity FLYWHEEL_HUB_SPEED = RPM.of(4000); // TODO: adjust
  public static final AngularVelocity FLYWHEEL_DEPOT_SPEED = RPM.of(4100);
  public static final AngularVelocity FLYWHEEL_ANYWHERE_SPEED = RPM.of(4100);
  public static final AngularVelocity FLYWHEEL_NEUTRAL_TO_ALLIANCE_SPEED = RPM.of(3700);
  public static final AngularVelocity FLYWHEEL_OPPONENT_TO_ALLIANCE_SPEED = RPM.of(6000);

  public static final double REVERSE_INTAKE_ROLLER_SPEED = -1;
  public static final double REVERSE_SERIALIZER_ROLLERS_SPEED = -1;
  public static final double REVERSE_FLYWHEEL_SPEED = -1;
  public static final double REVERSE_SHOOTER_TRANSFER_SPEED = -1;
  public static final double STOP = 0.0;
  static {

    INTAKE_ROLLER_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    INTAKE_ROLLER_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    INTAKE_ROLLER_CONFIGURATION.CurrentLimits.SupplyCurrentLimitEnable = true;
    INTAKE_ROLLER_CONFIGURATION.CurrentLimits.SupplyCurrentLowerLimit = 45;

    SERIALIZER_ROLLERS_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SERIALIZER_ROLLERS_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    SHOOTER_TRANSFER_EAST_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SHOOTER_TRANSFER_EAST_CONFIGURATION.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    SHOOTER_TRANSFER_EAST_CONFIGURATION.Slot0.kP = 0.7;
    SHOOTER_TRANSFER_EAST_CONFIGURATION.Slot0.kS = 0.15;
    SHOOTER_TRANSFER_EAST_CONFIGURATION.Slot0.kV = 0.12;
    SHOOTER_TRANSFER_EAST_CONFIGURATION.Slot0.kA = 0;
    SHOOTER_TRANSFER_EAST_CONFIGURATION.MotionMagic.MotionMagicCruiseVelocity = 0;
    SHOOTER_TRANSFER_EAST_CONFIGURATION.MotionMagic.MotionMagicAcceleration = 9999;
    SHOOTER_TRANSFER_EAST_CONFIGURATION.MotionMagic.MotionMagicJerk = 0;

    SHOOTER_TRANSFER_WEST_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SHOOTER_TRANSFER_WEST_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    FLYWHEEL_WEST_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FLYWHEEL_WEST_CONFIGURATION.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    FLYWHEEL_WEST_CONFIGURATION.Slot0.kS = 0.14;
    FLYWHEEL_WEST_CONFIGURATION.Slot0.kV = 0.11167;
    FLYWHEEL_WEST_CONFIGURATION.Slot0.kA = 0;
    FLYWHEEL_WEST_CONFIGURATION.Slot0.kP = 0.5;
    FLYWHEEL_WEST_CONFIGURATION.MotionMagic.MotionMagicCruiseVelocity = 0;
    FLYWHEEL_WEST_CONFIGURATION.MotionMagic.MotionMagicAcceleration = 9999;
    FLYWHEEL_WEST_CONFIGURATION.MotionMagic.MotionMagicJerk = 0;

    FLYWHEEL_EAST_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FLYWHEEL_EAST_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    FLYWHEEL_EAST_CONFIGURATION.Slot0.kS = 0.14;
    FLYWHEEL_EAST_CONFIGURATION.Slot0.kV = 0.11167;
    FLYWHEEL_EAST_CONFIGURATION.Slot0.kA = 0;
    FLYWHEEL_EAST_CONFIGURATION.Slot0.kP = 0.5;
    FLYWHEEL_EAST_CONFIGURATION.MotionMagic.MotionMagicCruiseVelocity = 0;
    FLYWHEEL_EAST_CONFIGURATION.MotionMagic.MotionMagicAcceleration = 9999;
    FLYWHEEL_EAST_CONFIGURATION.MotionMagic.MotionMagicJerk = 0;

    // flywheelSpeedMap.put(Inches.of(0.0).in(Inches), RPM.of(3200).in(RPM));
    // TODO: Consider re-adding a close-range (0 in) calibration point if required.
    // flywheelSpeedMap.put(Inches.of(60).in(Inches), RPM.of(3200).in(RPM));
    // flywheelSpeedMap.put(Inches.of(90).in(Inches), RPM.of(3600).in(RPM));
    // flywheelSpeedMap.put(Inches.of(120).in(Inches), RPM.of(4100).in(RPM));
    // flywheelSpeedMap.put(Inches.of(150).in(Inches), RPM.of(4300).in(RPM));
    // flywheelSpeedMap.put(Inches.of(187.8).in(Inches), RPM.of(5200).in(RPM));
    flywheelSpeedMap.put(Inches.of(187.8).in(Inches), RPM.of(4100).in(RPM));
    flywheelSpeedMap.put(Inches.of(165.1).in(Inches), RPM.of(4100).in(RPM));
    flywheelSpeedMap.put(Inches.of(165).in(Inches), RPM.of(4000).in(RPM));
    flywheelSpeedMap.put(Inches.of(0).in(Inches), RPM.of(4000).in(RPM));
  }
}