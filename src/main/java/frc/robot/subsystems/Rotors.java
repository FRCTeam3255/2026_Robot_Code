// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.epilogue.Logged;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs.rotorIDs;
import frc.robot.constants.ConstRotors;

@Logged
public class Rotors extends SubsystemBase {

  final TalonFX serializerRollers = new TalonFX(rotorIDs.SERIALIZER_ROLLERS_CAN);
  final TalonFX intakeRoller = new TalonFX(rotorIDs.INTAKE_ROLLERS_CAN);
  final TalonFX shooterTransferEast = new TalonFX(rotorIDs.SHOOTER_TRANSFER_EAST_CAN);
  final TalonFX shooterTransferWest = new TalonFX(rotorIDs.SHOOTER_TRANSFER_WEST_CAN);
  final TalonFX flywheelTopWest = new TalonFX(rotorIDs.FLYWHEEL_TOP_WEST_CAN);
  final TalonFX flywheelTopEast = new TalonFX(rotorIDs.FLYWHEEL_TOP_EAST_CAN);
  final TalonFX flywheelBottomWest = new TalonFX(rotorIDs.FLYWHEEL_BOTTOM_WEST_CAN);
  final TalonFX flywheelBottomEast = new TalonFX(rotorIDs.FLYWHEEL_BOTTOM_EAST_CAN);
  AngularVelocity lastDesiredFlywheelSpeed = Units.RPM.of(0);
  AngularVelocity lastDesiredTransferPercentOutput = Units.RPM.of(0);
  Follower flywheelEastFollower = new Follower(flywheelTopEast.getDeviceID(), MotorAlignmentValue.Aligned);
  Follower flywheelWestFollower = new Follower(flywheelTopWest.getDeviceID(), MotorAlignmentValue.Aligned);
  Follower shooterTransferFollower = new Follower(shooterTransferEast.getDeviceID(), MotorAlignmentValue.Opposed);

  /** Creates a new Rotors. */
  public Rotors() {
    serializerRollers.getConfigurator().apply(ConstRotors.SERIALIZER_ROLLERS_CONFIGURATION);
    intakeRoller.getConfigurator().apply(ConstRotors.INTAKE_ROLLER_CONFIGURATION);
    shooterTransferEast.getConfigurator().apply(ConstRotors.SHOOTER_TRANSFER_EAST_CONFIGURATION);
    shooterTransferWest.getConfigurator().apply(ConstRotors.SHOOTER_TRANSFER_WEST_CONFIGURATION);
    flywheelTopWest.getConfigurator().apply(ConstRotors.FLYWHEEL_WEST_CONFIGURATION);
    flywheelBottomWest.getConfigurator().apply(ConstRotors.FLYWHEEL_WEST_CONFIGURATION);
    flywheelTopEast.getConfigurator().apply(ConstRotors.FLYWHEEL_EAST_CONFIGURATION);
    flywheelBottomEast.getConfigurator().apply(ConstRotors.FLYWHEEL_EAST_CONFIGURATION);
  }

  final MotionMagicVelocityVoltage flywheelVelocityRequest = new MotionMagicVelocityVoltage(0);
  final MotionMagicVelocityVoltage shooterTransferVelocityRequest = new MotionMagicVelocityVoltage(0);

  public void setSerializerRollersSpeed(double speed) {
    serializerRollers.set(speed);
  }

  public void setIntakeRollersSpeed(double speed) {
    intakeRoller.set(speed);
  }

  public void setShooterTransferSpeed(AngularVelocity speed) {
    shooterTransferEast.setControl(shooterTransferVelocityRequest.withVelocity(speed));
    shooterTransferWest.setControl(shooterTransferVelocityRequest.withVelocity(speed));
    lastDesiredTransferPercentOutput = speed;
  }

  public void setFlywheelSpeed(AngularVelocity speed) {
    flywheelTopEast.setControl(flywheelVelocityRequest.withVelocity(speed));
    flywheelTopWest.setControl(flywheelWestFollower);
    flywheelBottomEast.setControl(flywheelEastFollower);
    flywheelBottomWest.setControl(flywheelWestFollower);
    lastDesiredFlywheelSpeed = speed;
  }

  public void setFlywheelPercentOutput(double percent) {
    flywheelTopEast.set(percent);
    flywheelTopWest.setControl(flywheelWestFollower);
    flywheelBottomEast.setControl(flywheelEastFollower);
    flywheelBottomWest.setControl(flywheelWestFollower);
    lastDesiredFlywheelSpeed = Units.RPM.of(5400 * percent);
  }

  public AngularVelocity getFlywheelSpeeds() {
    return flywheelBottomWest.getVelocity().getValue();
  }

  public boolean areFlywheelsAtSpeed(AngularVelocity tolerance) {
    AngularVelocity lowerLim = lastDesiredFlywheelSpeed.minus(tolerance);
    AngularVelocity upperLim = lastDesiredFlywheelSpeed.plus(tolerance);

    AngularVelocity flywheelSpeeds = getFlywheelSpeeds();

    return flywheelSpeeds.gte(lowerLim)
        && flywheelSpeeds.lte(upperLim);
  }

  public static AngularVelocity getMappedFlywheelSpeed(Distance distance) {
    double rpm = ConstRotors.flywheelSpeedMap.get(distance.in(Units.Inches));
    return RPM.of(rpm);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
