// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.PrepShoots;

import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstRotors;
import frc.robot.subsystems.StateMachine;

/** Add your docs here. */
public class PrepHub extends BasePrepShooter {
  public PrepHub() {
    super(ConstRotors.FLYWHEEL_HUB_SPEED, ConstMotion.HOOD_HUB_ANGLE,
        StateMachine.RobotState.PREP_HUB);

  }
}
