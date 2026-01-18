// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs.rotorIDs;
import frc.robot.constants.ConstRotors;

@Logged
public class Rotors extends SubsystemBase {

  final TalonFX serializerRollersMotor = new TalonFX(rotorIDs.SERIALIZER_ROLLERS_CAN);
  final TalonFX intakeRollerMotor = new TalonFX(rotorIDs.INTAKE_ROLLERS_CAN);
  final TalonFX serializerVFunnelMotor = new TalonFX(rotorIDs.SERIALIZER_V_FUNNEL_CAN);
  final TalonFX shooterTransferMotor = new TalonFX(rotorIDs.SHOOTER_TRANSFER_CAN);
  final TalonFX flywheelLeftMotor = new TalonFX(rotorIDs.FLYWHEEL_LEFT_CAN);
  final TalonFX flywheelRightMotor = new TalonFX(rotorIDs.FLYWHEEL_RIGHT_CAN);

  /** Creates a new Rotors. */
  public Rotors() {
    serializerRollersMotor.getConfigurator().apply(ConstRotors.SERIALIZER_ROLLERS_CONFIGURATION);
    intakeRollerMotor.getConfigurator().apply(ConstRotors.INTAKE_ROLLER_CONFIGURATION);
    serializerVFunnelMotor.getConfigurator().apply(ConstRotors.SERIALIZER_V_FUNNEL_CONFIGURATION);
    shooterTransferMotor.getConfigurator().apply(ConstRotors.SHOOTER_TRANSFER_CONFIGURATION);
    flywheelLeftMotor.getConfigurator().apply(ConstRotors.FLYWHEEL_LEFT_CONFIGURATION);
    flywheelLeftMotor.getConfigurator().apply(ConstRotors.FLYWHEEL_RIGHT_CONFIGURATION);
  }

  public void setSerializerRollersMotorSpeed(double speed) {
    serializerRollersMotor.set(speed);
  }

  public void setIntakeRollersMotorSpeed(double speed) {
    intakeRollerMotor.set(speed);
  }

  public void setShooterTransferMotorSpeed(double speed) {
    shooterTransferMotor.set(speed);
  }

  public void setSerializerVFunnelMotorSpeed(double speed) {
    serializerVFunnelMotor.set(speed);
  }

  public void setFlywheelMotorSpeed(double speed) {
    flywheelLeftMotor.set(speed);
    flywheelRightMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
