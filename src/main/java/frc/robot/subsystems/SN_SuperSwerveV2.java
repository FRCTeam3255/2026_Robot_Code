package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class SN_SuperSwerveV2 extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
  // ==========================================================
  // ****************** CTRE GENERATED CODE *******************
  // ************************ GLOBALS ************************
  // ==========================================================
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  // ==========================================================
  // ****************** CTRE GENERATED CODE *******************
  // ********************** CONSTRUCTORS **********************
  // NOTE: Originally Extended TunerConstants.TunerSwerveDrivetrain
  // Modified to extend SwerveDrivetrain directly
  // ==========================================================
  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * <p>
   * This constructs the underlying hardware devices, so users should not
   * construct
   * the devices themselves. If they need the devices, they can access them
   * through
   * getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules             Constants for each specific module
   */
  public SN_SuperSwerveV2(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        TalonFX::new, TalonFX::new, CANcoder::new,
        drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * <p>
   * This constructs the underlying hardware devices, so users should not
   * construct
   * the devices themselves. If they need the devices, they can access them
   * through
   * getters in the classes.
   *
   * @param drivetrainConstants       Drivetrain-wide constants for the swerve
   *                                  drive
   * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
   *                                  unspecified or set to 0 Hz, this is 250 Hz
   *                                  on
   *                                  CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry
   *                                  calculation
   *                                  in the form [x, y, theta]ᵀ, with units in
   *                                  meters
   *                                  and radians
   * @param visionStandardDeviation   The standard deviation for vision
   *                                  calculation
   *                                  in the form [x, y, theta]ᵀ, with units in
   *                                  meters
   *                                  and radians
   * @param modules                   Constants for each specific module
   */
  public SN_SuperSwerveV2(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        TalonFX::new, TalonFX::new, CANcoder::new,
        drivetrainConstants, odometryUpdateFrequency,
        odometryStandardDeviation, visionStandardDeviation, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  // ==========================================================
  // ****************** CTRE GENERATED CODE *******************
  // ************************ FUNCTIONS ************************
  // ==========================================================
  /**
   * Returns a command that applies the specified control request to this swerve
   * drivetrain.
   *
   * @param requestSupplier Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(allianceColor -> {
        setOperatorPerspectiveForward(
            allianceColor == Alliance.Red
                ? kRedAlliancePerspectiveRotation
                : kBlueAlliancePerspectiveRotation);
        m_hasAppliedOperatorPerspective = true;
      });
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the
   * odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision
   *                              camera.
   * @param timestampSeconds      The timestamp of the vision measurement in
   *                              seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the
   * odometry pose estimate
   * while still accounting for measurement noise.
   * <p>
   * Note that the vision measurement standard deviations passed into this method
   * will continue to apply to future measurements until a subsequent call to
   * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters    The pose of the robot as measured by the
   *                                 vision camera.
   * @param timestampSeconds         The timestamp of the vision measurement in
   *                                 seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose
   *                                 measurement
   *                                 in the form [x, y, theta]ᵀ, with units in
   *                                 meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
        visionMeasurementStdDevs);
  }

  // ==========================================================
  // ****************** CTRE GENERATED CODE *******************
  // ************************* SYSID *************************
  // ==========================================================
  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> setControl(m_translationCharacterization.withVolts(output)),
          null,
          this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> setControl(m_steerCharacterization.withVolts(volts)),
          null,
          this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> {
            /* output is actually radians per second, but SysId only supports "volts" */
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  // =========================================================
  // ********************** SUPERCORE ***********************
  // Code made by the SuperNURDs. Additions on to CTRE Swerve.
  // =========================================================
  // Swerve Requests were originally in CTRE Generated Swerve RobotContainer
  private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle();

  public void drive(ChassisSpeeds chassisSpeeds) {
    setControl(fieldCentricRequest
        .withVelocityX(chassisSpeeds.vxMetersPerSecond)
        .withVelocityY(chassisSpeeds.vyMetersPerSecond)
        .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));
  }

  public void drive(ChassisSpeeds chassisSpeeds, Rotation2d facingAngle, double kP, double kI, double kD) {
    setControl(fieldCentricFacingAngleRequest
        .withVelocityX(chassisSpeeds.vxMetersPerSecond)
        .withVelocityY(chassisSpeeds.vyMetersPerSecond)
        .withHeadingPID(kP, kI, kD)
        .withTargetDirection(facingAngle));
  }

  public void xBrake() {
    setControl(brakeRequest);
  }

  /**
   * @return The current rate of rotation for the Pigeon 2. <b>Units:</b> Degrees
   *         per Second
   */
  public AngularVelocity getGyroRate() {
    return this.getPigeon2().getAngularVelocityZWorld().getValue();
  }

  /**
   * Return the current estimated pose from the pose estimator.
   *
   * @return The current estimated pose
   */
  public Pose2d getPose() {
    return this.getState().Pose;
  }

  /**
   * Calculates the chassis velocities based on joystick inputs and other
   * parameters.
   *
   * @param xAxisSupplier        A DoubleSupplier providing the x-axis input for
   *                             forward/backward movement.
   * @param yAxisSupplier        A DoubleSupplier providing the y-axis input for
   *                             left/right movement.
   * @param rotationAxisSupplier A DoubleSupplier providing the rotation input for
   *                             turning.
   * @param slowMode             A BooleanSupplier indicating whether the slow
   *                             mode is active.
   * @param isRed                A boolean indicating if the robot is on the red
   *                             alliance (reverses controls if true).
   * @param SLOW_MODE_MULTIPLIER A multiplier applied to velocities when slow mode
   *                             is active.
   * @param REAL_DRIVE_SPEED     The maximum linear velocity of the robot in
   *                             meters per second.
   * @param TURN_SPEED           The maximum angular velocity of the robot in
   *                             radians per second.
   * @return A ChassisSpeeds object containing the calculated x, y, and rotational
   *         velocities.
   */
  public ChassisSpeeds calculateVelocitiesFromInput(
      DoubleSupplier xAxisSupplier,
      DoubleSupplier yAxisSupplier,
      DoubleSupplier rotationAxisSupplier,
      BooleanSupplier slowMode,
      boolean isRed,
      double SLOW_MODE_MULTIPLIER,
      LinearVelocity REAL_DRIVE_SPEED,
      AngularVelocity TURN_SPEED) {

    double redAllianceMultiplier = isRed ? -1 : 1;
    double slowModeMultiplier = slowMode.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0;

    double xVelocity = xAxisSupplier.getAsDouble() * REAL_DRIVE_SPEED.in(Units.MetersPerSecond)
        * redAllianceMultiplier * slowModeMultiplier;
    double yVelocity = -yAxisSupplier.getAsDouble() * REAL_DRIVE_SPEED.in(Units.MetersPerSecond)
        * redAllianceMultiplier * slowModeMultiplier;
    double rotationVelocity = -rotationAxisSupplier.getAsDouble()
        * TURN_SPEED.in(Units.RadiansPerSecond);

    return new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
  }

  /**
   * Determines whether the robot is within the specified auto-drive zone based on
   * the distance
   * to a target pose.
   *
   * @param autoDriveMaxDistance The maximum allowable distance for the auto-drive
   *                             zone. If null,
   *                             the method will return false.
   * @param target               The target pose to calculate the distance from
   *                             the robot's current pose.
   * @return True if the robot's current pose is within the specified maximum
   *         distance from the target pose,
   *         false otherwise.
   */
  public boolean isInAutoDriveZone(Distance autoDriveMaxDistance, Pose2d target) {
    if (autoDriveMaxDistance == null) {
      return false;
    }
    Distance distanceFromPose = Units.Meters
        .of(this.getPose().getTranslation().getDistance(target.getTranslation()));
    return distanceFromPose.lt(autoDriveMaxDistance);
  }

  public boolean isAtRotation(Rotation2d desiredRotation, Angle tolerance) {
    return (this.getPose().getRotation().getMeasure()
        .compareTo(desiredRotation.getMeasure().minus(tolerance)) > 0)
        && this.getPose().getRotation().getMeasure().compareTo(desiredRotation.getMeasure().plus(tolerance)) < 0;
  }

  public boolean isAtPosition(Pose2d desiredPose2d, Distance tolerance) {
    return Units.Meters.of(this.getPose().getTranslation().getDistance(desiredPose2d.getTranslation()))
        .lte(tolerance);
  }

}
