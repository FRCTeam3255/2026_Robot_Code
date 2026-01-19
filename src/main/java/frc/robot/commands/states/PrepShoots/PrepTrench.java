// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.PrepShoots;

import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstRotors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepTrench extends BasePrepShooter {
  public PrepTrench() {
    // TODO: change the drivetrain angle to correct value
    super(ConstRotors.FLYWHEEL_TRENCH_SPEED, ConstMotion.HOOD_TRENCH_ANGLE);
    // Use addRequirements() here to declare subsystem dependencies.
  }
}
