// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs.rotorIDs;
import frc.robot.constants.ConstRotors;

@Logged
public class Rotors extends SubsystemBase {

  final TalonFX serializerRollers = new TalonFX(rotorIDs.SERIALIZER_ROLLERS_CAN);
  final TalonFX intakeRoller = new TalonFX(rotorIDs.INTAKE_ROLLERS_CAN);
  final TalonFX serializerVFunnel = new TalonFX(rotorIDs.SERIALIZER_V_FUNNEL_CAN);
  final TalonFX shooterTransfer = new TalonFX(rotorIDs.SHOOTER_TRANSFER_CAN);
  final TalonFX flywheelTopLeft = new TalonFX(rotorIDs.FLYWHEEL_TOP_LEFT_CAN);
  final TalonFX flywheelTopRight = new TalonFX(rotorIDs.FLYWHEEL_TOP_RIGHT_CAN);
  final TalonFX flywheelBottomLeft = new TalonFX(rotorIDs.FLYWHEEL_BOTTOM_LEFT_CAN);
  final TalonFX flywheelBottomRight = new TalonFX(rotorIDs.FLYWHEEL_BOTTOM_RIGHT_CAN);

  /** Creates a new Rotors. */
  public Rotors() {
    serializerRollers.getConfigurator().apply(ConstRotors.SERIALIZER_ROLLERS_CONFIGURATION);
    intakeRoller.getConfigurator().apply(ConstRotors.INTAKE_ROLLER_CONFIGURATION);
    serializerVFunnel.getConfigurator().apply(ConstRotors.SERIALIZER_V_FUNNEL_CONFIGURATION);
    shooterTransfer.getConfigurator().apply(ConstRotors.SHOOTER_TRANSFER_CONFIGURATION);
    flywheelTopLeft.getConfigurator().apply(ConstRotors.FLYWHEEL_LEFT_CONFIGURATION);
    flywheelBottomLeft.getConfigurator().apply(ConstRotors.FLYWHEEL_LEFT_CONFIGURATION);
    flywheelTopRight.getConfigurator().apply(ConstRotors.FLYWHEEL_RIGHT_CONFIGURATION);
    flywheelBottomRight.getConfigurator().apply(ConstRotors.FLYWHEEL_RIGHT_CONFIGURATION);
  }

  final MotionMagicVelocityVoltage flywheelVelocityRequest = new MotionMagicVelocityVoltage(0);

  public void setSerializerRollersSpeed(double speed) {
    serializerRollers.set(speed);
  }

  public void setIntakeRollersSpeed(double speed) {
    intakeRoller.set(speed);
  }

  public void setShooterTransferSpeed(double speed) {
    shooterTransfer.set(speed);
  }

  public void setSerializerVFunnelSpeed(double speed) {
    serializerVFunnel.set(speed);
  }

  public void setFlywheelSpeed(double speed) {
    flywheelTopRight.set(speed);
    flywheelTopLeft.setControl(new Follower(flywheelTopRight.getDeviceID(), true));
    flywheelBottomRight.setControl(new Follower(flywheelTopRight.getDeviceID(), false));
    flywheelBottomLeft.setControl(new Follower(flywheelTopRight.getDeviceID(), true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
