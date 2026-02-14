// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConstRumble extends SubsystemBase {
  /** Creates a new ConstRumble. */

  public static final double READY_TO_SHOOT_RUMBLE = 0.5;
  public static final double SHIFT_CHANGE_RUMBLE = 1;
  public static final double RUMBLE_OFF = 0.0;

  public ConstRumble() {
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
