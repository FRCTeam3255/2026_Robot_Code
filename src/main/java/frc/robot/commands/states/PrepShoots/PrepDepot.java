// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.PrepShoots;

import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstPoseDrive;
import frc.robot.constants.ConstRotors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepDepot extends BasePrepShooter {
  public PrepDepot() {
    super(ConstRotors.FLYWHEEL_DEPOT_SPEED, ConstMotion.HOOD_DEPOT_ANGLE,
        ConstPoseDrive.PrepShootRotations.DEPOT_PREP_SHOOT_ROTATION);
    // Use addRequirements() here to declare subsystem dependencies.
  }
}
