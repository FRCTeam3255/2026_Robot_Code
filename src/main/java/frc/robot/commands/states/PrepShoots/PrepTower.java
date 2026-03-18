// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.PrepShoots;

import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstRotors;
import frc.robot.subsystems.StateMachine;

/** Add your docs here. */
public class PrepTower extends BasePrepShooter {
  public PrepTower() {
    super(ConstRotors.FLYWHEEL_TOWER_SPEED, ConstMotion.HOOD_TOWER_ANGLE,
        StateMachine.RobotState.PREP_TOWER);
  }
}