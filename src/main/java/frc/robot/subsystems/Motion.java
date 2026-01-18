// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs;

@Logged
public class Motion extends SubsystemBase {
  /** Creates a new Motion. */
  final TalonFX IntakePivot = new TalonFX(DeviceIDs.motionIDs.INTAKE_PIVOT_CAN);
  final TalonFX Climber = new TalonFX(DeviceIDs.motionIDs.CLIMBER_CAN);
  final TalonFX Hood = new TalonFX(DeviceIDs.motionIDs.HOOD_CAN);

  public Motion() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void pivotIntake(double speed) {
    IntakePivot.set(speed);
  }

  public void startClimb(double speed) {
    Climber.set(speed);
  }

  public void pivotHood(double speed) {
    Hood.set(speed);
  }
}
