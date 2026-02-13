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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs.rotorIDs;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstRotors;

@Logged
public class Rotors extends SubsystemBase {

  final TalonFX serializerRollers = new TalonFX(rotorIDs.SERIALIZER_ROLLERS_CAN);
  final TalonFX intakeRoller = new TalonFX(rotorIDs.INTAKE_ROLLERS_CAN);
  final TalonFX serializerVFunnel = new TalonFX(rotorIDs.SERIALIZER_V_FUNNEL_CAN);
  final TalonFX shooterTransfer = new TalonFX(rotorIDs.SHOOTER_TRANSFER_CAN);
  final TalonFX flywheelTopWest = new TalonFX(rotorIDs.FLYWHEEL_TOP_WEST_CAN);
  final TalonFX flywheelTopEast = new TalonFX(rotorIDs.FLYWHEEL_TOP_EAST_CAN);
  final TalonFX flywheelBottomWest = new TalonFX(rotorIDs.FLYWHEEL_BOTTOM_WEST_CAN);
  final TalonFX flywheelBottomEast = new TalonFX(rotorIDs.FLYWHEEL_BOTTOM_EAST_CAN);
  public AngularVelocity lastDesiredFlywheelSpeed = Units.RPM.of(0);

  /** Creates a new Rotors. */
  public Rotors() {
    serializerRollers.getConfigurator().apply(ConstRotors.SERIALIZER_ROLLERS_CONFIGURATION);
    intakeRoller.getConfigurator().apply(ConstRotors.INTAKE_ROLLER_CONFIGURATION);
    serializerVFunnel.getConfigurator().apply(ConstRotors.SERIALIZER_V_FUNNEL_CONFIGURATION);
    shooterTransfer.getConfigurator().apply(ConstRotors.SHOOTER_TRANSFER_CONFIGURATION);
    flywheelTopWest.getConfigurator().apply(ConstRotors.FLYWHEEL_WEST_CONFIGURATION);
    flywheelBottomWest.getConfigurator().apply(ConstRotors.FLYWHEEL_WEST_CONFIGURATION);
    flywheelTopEast.getConfigurator().apply(ConstRotors.FLYWHEEL_EAST_CONFIGURATION);
    flywheelBottomEast.getConfigurator().apply(ConstRotors.FLYWHEEL_EAST_CONFIGURATION);
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

  public void setFlywheelSpeed(AngularVelocity speed) {
    flywheelTopEast.setControl(flywheelVelocityRequest.withVelocity(speed));
    flywheelTopWest.setControl(flywheelVelocityRequest.withVelocity(speed));
    flywheelBottomEast.setControl(new Follower(flywheelTopEast.getDeviceID(), MotorAlignmentValue.Aligned));
    flywheelBottomWest.setControl(new Follower(flywheelTopWest.getDeviceID(), MotorAlignmentValue.Aligned));
    lastDesiredFlywheelSpeed = speed;
  }

  public void setFlywheelPercentOutput(double percent) {
    flywheelTopEast.set(percent);
    flywheelTopWest.set(percent);
    flywheelBottomEast.set(percent);
    flywheelBottomWest.set(percent);
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
