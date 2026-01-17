// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.DeviceIDs;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstPoseDrive.PoseDriveGroup;

public class Drivetrain extends SN_SuperSwerveV2 {

  public PoseDriveGroup lastDesiredPoseGroup;
  public Pose2d lastDesiredTarget;

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
          ConstDrivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET,
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
          ConstDrivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET,
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
          ConstDrivetrain.BACK_LEFT_ABS_ENCODER_OFFSET,
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
          ConstDrivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET,
          ConstDrivetrain.MODULE_OFFSET_LOCATIONS.unaryMinus(),
          ConstDrivetrain.MODULE_OFFSET_LOCATIONS.unaryMinus(),
          ConstDrivetrain.INVERT_RIGHT_SIDE_DRIVE,
          ConstDrivetrain.INVERT_STEER,
          ConstDrivetrain.INVERT_STEER_ENCODER);
  public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
      .withCANBusName(DeviceIDs.drivetrainIDs.CAN_BUS_NAME.getName())
      .withPigeon2Id(DeviceIDs.drivetrainIDs.PIGEON_CAN)
      .withPigeon2Configs(ConstDrivetrain.pigeonConfigs);

  public Drivetrain() {
    super(
        DrivetrainConstants,
        FrontLeft,
        FrontRight,
        BackLeft,
        BackRight);
  }

  /**
   * Follows a trajectory by calculating the desired chassis speeds based on the
   * current pose
   * of the robot and the target pose provided in the trajectory sample.
   *
   * @param sample The trajectory sample containing the desired target pose and
   *               other relevant data.
   *               This is used to determine the robot's next movement.
   */
  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d desiredTarget = sample.getPose();
    ChassisSpeeds automatedDTVelocity = ConstDrivetrain.AUTO_ALIGN.PATH_AUTO_ALIGN_CONTROLLER.calculate(getPose(),
        desiredTarget, 0,
        desiredTarget.getRotation());

    drive(automatedDTVelocity);
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

}
