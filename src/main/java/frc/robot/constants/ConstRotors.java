// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ConstRotors {

  public static final TalonFXConfiguration SERIALIZER_ROLLERS_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration INTAKE_ROLLER_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration SERIALIZER_V_FUNNEL_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration SHOOTER_TRANSFER_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration FLYWHEEL_LEFT_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration FLYWHEEL_RIGHT_CONFIGURATION = new TalonFXConfiguration();
  public static final double STOP = 0.0;
  public static final double INTAKE_ROLLER_SPEED = 1;
  public static final double SERIALIZER_V_FUNNEL_SPEED = 1;
  public static final double SERIALIZER_ROLLERS_SPEED = 1;
  public static final double SHOOTER_TRANSFER_SPEED = 1;
  public static final double FLYWHEEL_SPEED = 1;
  public static final double REVERSE_INTAKE_ROLLER_SPEED = -1;
  public static final double REVERSE_SERIALIZER_V_FUNNEL_SPEED = -1;
  public static final double REVERSE_SERIALIZER_ROLLERS_SPEED = -1;
  public static final double REVERSE_FLYWHEEL_SPEED = -1;
  public static final double REVERSE_SHOOTER_TRANSFER_SPEED = -1;

  static {
    INTAKE_ROLLER_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    INTAKE_ROLLER_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    SERIALIZER_ROLLERS_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SERIALIZER_ROLLERS_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    SERIALIZER_V_FUNNEL_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SERIALIZER_V_FUNNEL_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    FLYWHEEL_LEFT_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FLYWHEEL_LEFT_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    FLYWHEEL_RIGHT_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FLYWHEEL_RIGHT_CONFIGURATION.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    SHOOTER_TRANSFER_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    SHOOTER_TRANSFER_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }
}