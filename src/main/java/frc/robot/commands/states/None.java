// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstLEDs;
import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstRotors;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class None extends Command {
  /** Creates a new None. */

  public None() {
    addRequirements(RobotContainer.stateMachineInstance);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.motionInstance.setClimberPosition(ConstMotion.RETRACT_CLIMBER);
    RobotContainer.motionInstance.setHoodAngle(ConstMotion.HOOD_NONE_ANGLE);
    RobotContainer.rotorsInstance.setFlywheelPercentOutput(ConstRotors.STOP);
    RobotContainer.rotorsInstance.setShooterTransferSpeed(ConstRotors.STOP);
    RobotContainer.rotorsInstance.setIntakeRollersSpeed(ConstRotors.STOP);
    RobotContainer.rotorsInstance.setSerializerVFunnelSpeed(ConstRotors.STOP);
    RobotContainer.rotorsInstance.setSerializerRollersSpeed(ConstRotors.STOP);
    RobotContainer.stateMachineInstance.setRobotState(StateMachine.RobotState.NONE);
    RobotContainer.LEDInstance.setLEDAnimation(ConstLEDs.NONE_ANIMATION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
