// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.PrepShoots;

import frc.robot.RobotContainer;
import frc.robot.constants.ConstLEDs;
import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstPoseDrive;
import frc.robot.constants.ConstRotors;
import frc.robot.subsystems.StateMachine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepTrench extends BasePrepShooter {
  public PrepTrench() {
    // TODO: change the drivetrain angle to correct value
    super(ConstRotors.FLYWHEEL_TRENCH_SPEED, ConstMotion.HOOD_TRENCH_ANGLE,
        ConstPoseDrive.PrepShootRotations.TRENCH_PREP_SHOOT_ROTATION, StateMachine.RobotState.PREP_TRENCH);
    if (RobotContainer.rotorsInstance.areFlywheelsAtSpeed(ConstRotors.FLYWHEEL_TOLERANCE)
        && RobotContainer.motionInstance.isHoodAtPosition(ConstMotion.HOOD_TOLERANCE)) {
      RobotContainer.LEDInstance.setLEDAnimation(ConstLEDs.READY_TO_SHOOT_ANIMATION);
    }    // Use addRequirements() here to declare subsystem dependencies.
  }
}
