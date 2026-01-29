// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ConstMotion {
  public static final TalonFXConfiguration INTAKE_PIVOT_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration CLIMBER_CONFIGURATION = new TalonFXConfiguration();
  public static final TalonFXConfiguration HOOD_CONFIGURATION = new TalonFXConfiguration();
  public static final double STOP = 0.0;
  public static final Angle DEPLOY_INTAKE_PIVOT_ANGLE = Units.Degrees.of(0); // TODO: set angle for intake pivot
  public static final Angle RETRACT_INTAKE_PIVOT_ANGLE = Units.Degrees.of(0); // TODO: set angle for intake pivot
  public static final double EXTEND_CLIMBER_SPEED = 0;
  public static final double RETRACT_CLIMBER_SPEED = -0.5;
  public static final double ANGLE_HOOD_SPEED = 0.5;
  public static final Angle HOOD_TRENCH_ANGLE = Units.Degrees.of(5);
  public static final Angle HOOD_OUTPOST_ANGLE = Units.Degrees.of(5);
  public static final Angle HOOD_NON_OUTPOST_ANGLE = Units.Degrees.of(5);
  public static final Angle HOOD_DEPOT_ANGLE = Units.Degrees.of(5);
  public static final Angle HOOD_ANYWHERE_ANGLE = Units.Degrees.of(5);
  public static final Angle HOOD_NUETRAL_TO_ALLIANCE_ANGLE = Units.Degrees.of(5);
  public static final Angle HOOD_OPPENENT_TO_ALLIANCE_ANGLE = Units.Degrees.of(5);
  public static final Angle HOOD_NONE_ANGLE = Units.Degrees.of(0);
  public static final Distance RETRACT_CLIMBER = Inches.of(0);
  public static final Distance EXTEND_CLIMBER = Inches.of(0);
  static {

    INTAKE_PIVOT_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    INTAKE_PIVOT_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    CLIMBER_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    CLIMBER_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    HOOD_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    HOOD_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }

  public class ShooterHoodTable {

    public final static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    public ShooterHoodTable() {
      hoodAngleMap.put(Inches.of(1.5).in(Inches), Degrees.of(28.0).in(Degrees));
      hoodAngleMap.put(Inches.of(2.0).in(Inches), Degrees.of(31.0).in(Degrees));
      hoodAngleMap.put(Inches.of(2.5).in(Inches), Degrees.of(35.0).in(Degrees));
      hoodAngleMap.put(Inches.of(3.0).in(Inches), Degrees.of(39.0).in(Degrees));
      hoodAngleMap.put(Inches.of(3.5).in(Inches), Degrees.of(43.0).in(Degrees));
      hoodAngleMap.put(Inches.of(4.0).in(Inches), Degrees.of(47.0).in(Degrees));
    }

    public static Angle getHoodAngle(Distance distance) {
      return Degrees.of(hoodAngleMap.get((Double) distance.in(Inches)));
    }
  }

}