// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.frcteam3255.components.swerve.SN_SuperSwerveV2;
import frc.robot.constants.ConstField;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.robot.DeviceIDs;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstPoseDrive.PoseDriveGroup;

@Logged
public class Drivetrain extends SN_SuperSwerveV2 {

  public PoseDriveGroup lastDesiredPoseGroup;
  public Pose2d lastDesiredTarget;
  private Rotation2d driveRotation = new Rotation2d();
  private double manualDriveRotation = 0.0;
  private boolean manualRotationEnabled = true;

  /** Creates a new Drivetrain. */
  public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
      .withDriveMotorGearRatio(ConstDrivetrain.GEAR_RATIOS.getDrive())
      .withSteerMotorGearRatio(ConstDrivetrain.GEAR_RATIOS.getSteer())
      .withCouplingGearRatio(ConstDrivetrain.GEAR_RATIOS.getCouple())
      .withWheelRadius(ConstDrivetrain.WHEEL_DIAMETER.div(2))
      .withSteerMotorGains(ConstDrivetrain.STEER_CONFIG)
      .withDriveMotorGains(ConstDrivetrain.DRIVE_CONFIG)
      .withSteerMotorClosedLoopOutput(ConstDrivetrain.closedLoopOutputType)
      .withDriveMotorClosedLoopOutput(ConstDrivetrain.closedLoopOutputType)
      .withSlipCurrent(ConstDrivetrain.WHEEL_SLIP_STATOR_CURRENT)
      .withSpeedAt12Volts(ConstDrivetrain.REAL_DRIVE_SPEED)
      .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
      .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
      .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
      .withDriveMotorInitialConfigs(ConstDrivetrain.driveInitialConfigs)
      .withSteerMotorInitialConfigs(ConstDrivetrain.steerInitialConfigs)
      .withEncoderInitialConfigs(ConstDrivetrain.encoderInitialConfigs)
      .withSteerInertia(ConstDrivetrain.SIMULATION.kSteerInertia)
      .withDriveInertia(ConstDrivetrain.SIMULATION.kDriveInertia)
      .withSteerFrictionVoltage(ConstDrivetrain.SIMULATION.kSteerFrictionVoltage)
      .withDriveFrictionVoltage(ConstDrivetrain.SIMULATION.kDriveFrictionVoltage);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = constantCreator
      .createModuleConstants(
          DeviceIDs.drivetrainIDs.FRONT_LEFT_STEER_CAN,
          DeviceIDs.drivetrainIDs.FRONT_LEFT_DRIVE_CAN,
          DeviceIDs.drivetrainIDs.FRONT_LEFT_ABSOLUTE_ENCODER_CAN,
          (RobotContainer.isPracticeBot()) ? ConstDrivetrain.PRACTICE_BOT.FRONT_LEFT_ABS_ENCODER_OFFSET
              : ConstDrivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET,
          ConstDrivetrain.MODULE_OFFSET_LOCATIONS,
          ConstDrivetrain.MODULE_OFFSET_LOCATIONS,
          ConstDrivetrain.INVERT_LEFT_SIDE_DRIVE,
          ConstDrivetrain.INVERT_STEER,
          ConstDrivetrain.INVERT_STEER_ENCODER);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = constantCreator
      .createModuleConstants(
          DeviceIDs.drivetrainIDs.FRONT_RIGHT_STEER_CAN,
          DeviceIDs.drivetrainIDs.FRONT_RIGHT_DRIVE_CAN,
          DeviceIDs.drivetrainIDs.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN,
          (RobotContainer.isPracticeBot()) ? ConstDrivetrain.PRACTICE_BOT.FRONT_RIGHT_ABS_ENCODER_OFFSET
              : ConstDrivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET,
          ConstDrivetrain.MODULE_OFFSET_LOCATIONS,
          ConstDrivetrain.MODULE_OFFSET_LOCATIONS.unaryMinus(),
          ConstDrivetrain.INVERT_RIGHT_SIDE_DRIVE,
          ConstDrivetrain.INVERT_STEER,
          ConstDrivetrain.INVERT_STEER_ENCODER);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = constantCreator
      .createModuleConstants(
          DeviceIDs.drivetrainIDs.BACK_LEFT_STEER_CAN,
          DeviceIDs.drivetrainIDs.BACK_LEFT_DRIVE_CAN,
          DeviceIDs.drivetrainIDs.BACK_LEFT_ABSOLUTE_ENCODER_CAN,
          (RobotContainer.isPracticeBot()) ? ConstDrivetrain.PRACTICE_BOT.BACK_LEFT_ABS_ENCODER_OFFSET
              : ConstDrivetrain.BACK_LEFT_ABS_ENCODER_OFFSET,
          ConstDrivetrain.MODULE_OFFSET_LOCATIONS.unaryMinus(),
          ConstDrivetrain.MODULE_OFFSET_LOCATIONS,
          ConstDrivetrain.INVERT_LEFT_SIDE_DRIVE,
          ConstDrivetrain.INVERT_STEER,
          ConstDrivetrain.INVERT_STEER_ENCODER);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = constantCreator
      .createModuleConstants(
          DeviceIDs.drivetrainIDs.BACK_RIGHT_STEER_CAN,
          DeviceIDs.drivetrainIDs.BACK_RIGHT_DRIVE_CAN,
          DeviceIDs.drivetrainIDs.BACK_RIGHT_ABSOLUTE_ENCODER_CAN,
          (RobotContainer.isPracticeBot()) ? ConstDrivetrain.PRACTICE_BOT.BACK_RIGHT_ABS_ENCODER_OFFSET
              : ConstDrivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET,
          ConstDrivetrain.MODULE_OFFSET_LOCATIONS.unaryMinus(),
          ConstDrivetrain.MODULE_OFFSET_LOCATIONS.unaryMinus(),
          ConstDrivetrain.INVERT_RIGHT_SIDE_DRIVE,
          ConstDrivetrain.INVERT_STEER,
          ConstDrivetrain.INVERT_STEER_ENCODER);
  public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
      .withCANBusName(DeviceIDs.drivetrainIDs.CAN_BUS_NAME.getName())
      .withPigeon2Id(DeviceIDs.drivetrainIDs.PIGEON_CAN)
      .withPigeon2Configs(ConstDrivetrain.pigeonConfigs);

  // Exposed motors for Epilogue logging
  public final TalonFX FrontLeftDrive;
  public final TalonFX FrontLeftSteer;
  public final TalonFX FrontRightDrive;
  public final TalonFX FrontRightSteer;
  public final TalonFX BackLeftDrive;
  public final TalonFX BackLeftSteer;
  public final TalonFX BackRightDrive;
  public final TalonFX BackRightSteer;

  public Drivetrain() {
    super(
        DrivetrainConstants,
        FrontLeft,
        FrontRight,
        BackLeft,
        BackRight);

    // Initialize motor references for Epilogue logging
    // Front Left (index 0)
    FrontLeftDrive = getModule(0).getDriveMotor();
    FrontLeftSteer = getModule(0).getSteerMotor();
    // Front Right (index 1)
    FrontRightDrive = getModule(1).getDriveMotor();
    FrontRightSteer = getModule(1).getSteerMotor();
    // Back Left (index 2)
    BackLeftDrive = getModule(2).getDriveMotor();
    BackLeftSteer = getModule(2).getSteerMotor();
    // Back Right (index 3)
    BackRightDrive = getModule(3).getDriveMotor();
    BackRightSteer = getModule(3).getSteerMotor();
  }

  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds = new ChassisSpeeds(
        sample.vx + ConstDrivetrain.AUTO_ALIGN.POSE_TRANS_CONTROLLER.calculate(pose.getX(), sample.x),
        sample.vy + ConstDrivetrain.AUTO_ALIGN.POSE_TRANS_CONTROLLER.calculate(pose.getY(), sample.y),
        sample.omega + ConstDrivetrain.AUTO_ALIGN.POSE_ROTATION_CONTROLLER.calculate(pose.getRotation().getRadians(),
            sample.heading));
    drive(speeds);
  }

  public void rotationalAlign(Pose2d closestPose, ChassisSpeeds velocities) {
    ProfiledPIDController autoAlignRotationPID = ConstDrivetrain.AUTO_ALIGN.POSE_ROTATION_CONTROLLER;
    drive(
        velocities,
        closestPose.getRotation(),
        autoAlignRotationPID.getP(),
        autoAlignRotationPID.getI(),
        autoAlignRotationPID.getD());
  }

  public void autoAlign(
      Pose2d desiredTarget,
      ChassisSpeeds manualVelocities,
      boolean lockX,
      boolean lockY) {

    // Full auto-align
    ChassisSpeeds automatedDTVelocity = ConstDrivetrain.AUTO_ALIGN.POSE_AUTO_ALIGN_CONTROLLER.calculate(getPose(),
        desiredTarget, 0,
        desiredTarget.getRotation());

    if (lockX) {
      automatedDTVelocity.vxMetersPerSecond = manualVelocities.vxMetersPerSecond;
    }
    if (lockY) {
      automatedDTVelocity.vyMetersPerSecond = manualVelocities.vyMetersPerSecond;
    }
    drive(automatedDTVelocity);
  }

  public boolean isRotationStickHit(DoubleSupplier rotationXAxis, DoubleSupplier rotationYAxis) {
    double rightStickX = rotationXAxis.getAsDouble();
    double rightStickY = rotationYAxis.getAsDouble();
    double hypotenuse = Math.hypot(rightStickX, rightStickY);

    return (hypotenuse < 1.15 && hypotenuse > 0.85);
  }

  public double getStickRadians(DoubleSupplier rotationXAxis, DoubleSupplier rotationYAxis) {
    double rightStickX = rotationXAxis.getAsDouble();
    double rightStickY = rotationYAxis.getAsDouble();
    double hypotenuse = Math.hypot(rightStickX, rightStickY);

    if (hypotenuse < 1.15 && hypotenuse > 0.85) {
      manualDriveRotation = Math.atan2(rightStickY, rightStickX) - Math.PI / 2;
    }
    return manualDriveRotation;
  }

  public Pose2d getDesiredTower() {
    List<Pose2d> TowerPoses = ConstField.getTowerPositions().get();
    Pose2d currentPose = getPose();
    Pose2d desiredTower = currentPose.nearest(TowerPoses);

    return desiredTower;
  }

  public void setDriveRotation(Angle rotation) {
    this.driveRotation = Rotation2d.fromDegrees(rotation.in(Degrees));
  }

  public Rotation2d getDriveRotation() {
    return this.driveRotation;
  }

  public Angle snapToTarget(Pose2d targetPose) {
    double dx = targetPose.getX() - getPose().getX();
    double dy = targetPose.getY() - getPose().getY();
    double angleRad = Math.atan2(dy, dx);
    return Degrees.of(Math.toDegrees(angleRad));
  }

  public boolean isAtDesiredRotation(Angle tolerance) {
    Angle upperLim = getDriveRotation().getMeasure().plus(tolerance);
    Angle lowerLim = getDriveRotation().getMeasure().plus(tolerance);
    return getPose().getRotation().getMeasure().gte(lowerLim)
        && getPose().getRotation().getMeasure().lte(upperLim);
  }

  public boolean isManualRotationEnabled() {
    return manualRotationEnabled;
  }

  public void setIsManualRotationEnabled(boolean set) {
    manualRotationEnabled = set;
  }
}
