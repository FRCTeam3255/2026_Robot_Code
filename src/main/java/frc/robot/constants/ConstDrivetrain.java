// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

/**
 * The {@code ConstDrivetrain} class serves as a centralized repository for all
 * configuration constants related to the robot's drivetrain subsystem.
 * <p>
 * This class contains static final fields for hardware configuration (such as
 * absolute encoder offsets, gear ratios, wheel dimensions), control parameters
 * (such as speed multipliers, inversion flags), and physical measurements
 * (such as track width and wheelbase). These constants are used throughout the
 * robot code to ensure consistent and maintainable configuration of the
 * drivetrain.
 * <p>
 * Usage: Reference these constants statically wherever drivetrain configuration
 * values are required. This approach helps avoid magic numbers and makes it
 * easier to update configuration values in a single location.
 */
public class ConstDrivetrain {
  // TODO: Swoffsets
  public static final Angle FRONT_LEFT_ABS_ENCODER_OFFSET = Rotations.of(-0.178466796875);
  public static final Angle FRONT_RIGHT_ABS_ENCODER_OFFSET = Rotations.of(-0.498779296875);
  public static final Angle BACK_LEFT_ABS_ENCODER_OFFSET = Rotations.of(-0.459716796875);
  public static final Angle BACK_RIGHT_ABS_ENCODER_OFFSET = Rotations.of(-0.31201171875);

  // ====== TO MOVE TO SUPERCODE - START ======
  public static class ModuleLocations {
    public static final Distance frame25x25 = Inches.of(19.75).div(2);
    public static final Distance frame29x29 = Inches.of(23.75).div(2);
  }

  public static class Ratios {
    private final double steer;
    private final double drive;
    private final double couple;

    public Ratios(double steer, double drive, double couple) {
      this.steer = steer;
      this.drive = drive;
      this.couple = couple;
    }

    public double getSteer() {
      return steer;
    }

    public double getDrive() {
      return drive;
    }

    public double getCouple() {
      return couple;
    }

    public static class MK4I {
      private static final double stage1 = 1. / (14. / 50.);
      private static final double stage2L1 = 1. / (25. / 19.);
      private static final double stage2L2 = 1. / (27. / 17.);
      private static final double stage2L3 = 1. / (28. / 16.);
      private static final double stage3 = 1. / (15. / 45.);
      private static final double steer = 150. / 7.;

      private static final double driveL1 = stage1 * stage2L1 * stage3;
      private static final double driveL2 = stage1 * stage2L2 * stage3;
      private static final double driveL3 = stage1 * stage2L3 * stage3;
      private static final double couple = stage1;

      public static final Ratios L1 = new Ratios(steer, driveL1, couple);
      public static final Ratios L2 = new Ratios(steer, driveL2, couple);
      public static final Ratios L3 = new Ratios(steer, driveL3, couple);
    }
  }
  // ====== TO MOVE TO SUPERCODE - END ======

  public static final double SLOW_MODE_MULTIPLIER = 0.5;

  // Physical dimensions
  public static Ratios GEAR_RATIOS = Ratios.MK4I.L2;

  public static Distance WHEEL_DIAMETER = Inches.of(4.0);
  public static Distance MODULE_OFFSET_LOCATIONS = ModuleLocations.frame29x29;

  /**
   * Follow the instructions in <a href=
   * "https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html#preventing-wheel-slip">
   * Preventing Wheel Slip</a> to find the slip current of the drivetrain.
   */
  public static final Current WHEEL_SLIP_STATOR_CURRENT = Amps.of(120);

  // Theoretical free speed (m/s) at 12 V applied output;
  // TODO: This needs to be tuned to your individual robot
  // TODO: DETERMINE HOW THIS IS GATHERED
  public static final LinearVelocity REAL_DRIVE_SPEED = MetersPerSecond.of(4.49);

  // -- Pose Estimation --
  /**
   * <p>
   * Pose estimator standard deviation for encoder & gyro data
   * </p>
   * <b>Units:</b> Meters
   */
  public static final double MEASUREMENT_STD_DEVS_POS = 0.05;

  /**
   * <p>
   * Pose estimator standard deviation for encoder & gyro data
   * </p>
   * <b>Units:</b> Radians
   */
  public static final double MEASUREMENT_STD_DEV_HEADING = Units.Radians.convertFrom(5, Units.Degrees);

  // Inverts
  public static final boolean INVERT_LEFT_SIDE_DRIVE = false;
  public static final boolean INVERT_RIGHT_SIDE_DRIVE = true;
  public static final boolean INVERT_STEER = true;
  public static final boolean INVERT_STEER_ENCODER = false;

  // -- CONFIGS --
  // Initial configs for the drive and steer motors and the azimuth encoder; these
  // cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API
  // documentation.
  public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(
          new CurrentLimitsConfigs()
              // Swerve azimuth does not require much torque output, so we can set a
              // relatively low
              // stator current limit to help avoid brownouts without impacting performance.
              .withStatorCurrentLimit(Amps.of(60))
              .withStatorCurrentLimitEnable(true));
  public static final CANcoderConfiguration encoderInitialConfigs = null;
  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  public static final Pigeon2Configuration pigeonConfigs = null;
  public static Slot0Configs DRIVE_CONFIG = new Slot0Configs();
  public static Slot0Configs STEER_CONFIG = new Slot0Configs();

  public static ClosedLoopOutputType closedLoopOutputType = ClosedLoopOutputType.Voltage;

  // Rotational speed (degrees per second) while manually driving
  public static final AngularVelocity TURN_SPEED = Units.DegreesPerSecond.of(360);

  // -- Motor Configurations --
  static {
    // This PID is implemented on each module, not the Drivetrain subsystem.
    // TODO: PID
    DRIVE_CONFIG.kP = 0.1;
    DRIVE_CONFIG.kI = 0.0;
    DRIVE_CONFIG.kD = 0.0;
    DRIVE_CONFIG.kS = 0.0;
    DRIVE_CONFIG.kV = 0.124;

    STEER_CONFIG.kP = 100;
    STEER_CONFIG.kI = 0.0;
    STEER_CONFIG.kD = 0.5;
    STEER_CONFIG.kS = 0.1;
    STEER_CONFIG.kV = 2.66;
    STEER_CONFIG.kA = 0.0;
    STEER_CONFIG.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
  }

  public static class AUTO_ALIGN {
    public static final LinearVelocity MIN_DRIVER_OVERRIDE = ConstDrivetrain.REAL_DRIVE_SPEED.div(10);

    public static final PIDController POSE_TRANS_CONTROLLER = new PIDController(
        3,
        0,
        0);

    public static final PIDController PATH_TRANS_CONTROLLER = new PIDController(
        3,
        0,
        0);

    public static final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.5);

    public static final ProfiledPIDController POSE_ROTATION_CONTROLLER = new ProfiledPIDController(
        2, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(Units.DegreesPerSecond),
            Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));

    public static final ProfiledPIDController PATH_ROTATION_CONTROLLER = new ProfiledPIDController(
        1, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(Units.DegreesPerSecond),
            Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));

    public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

    static {
      POSE_TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

      POSE_ROTATION_CONTROLLER.enableContinuousInput(0, 360);
      POSE_ROTATION_CONTROLLER.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));
    }

    public static HolonomicDriveController POSE_AUTO_ALIGN_CONTROLLER = new HolonomicDriveController(
        POSE_TRANS_CONTROLLER,
        POSE_TRANS_CONTROLLER,
        POSE_ROTATION_CONTROLLER);

    public static HolonomicDriveController PATH_AUTO_ALIGN_CONTROLLER = new HolonomicDriveController(
        PATH_TRANS_CONTROLLER,
        PATH_TRANS_CONTROLLER,
        PATH_ROTATION_CONTROLLER);

  }

  public static class SIMULATION {
    // These are only used for simulation
    public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);
  }

}