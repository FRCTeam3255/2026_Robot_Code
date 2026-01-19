// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.PrepShoots;

import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstRotors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepOpponentToAlliance extends BasePrepShooter {
  public PrepOpponentToAlliance() {
    super(ConstRotors.FLYWHEEL_OPPONENT_TO_ALLIANCE_SPEED, ConstMotion.HOOD_OPPENENT_TO_ALLIANCE_ANGLE);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}
