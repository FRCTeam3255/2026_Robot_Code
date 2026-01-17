// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;

public final class ConstSystem {
  /**
   * Volts
   */

  public static final Transform3d ROBOT_TO_BUMPERS = new Transform3d(0, 0, Units.Meters.convertFrom(5, Units.Inches),
      Rotation3d.kZero); // TODO: Replace with actual measurement

  public static class constControllers {
    public static final double DRIVER_LEFT_STICK_DEADBAND = 0.05;
    public static final boolean SILENCE_JOYSTICK_WARNINGS = true;
  }

}
