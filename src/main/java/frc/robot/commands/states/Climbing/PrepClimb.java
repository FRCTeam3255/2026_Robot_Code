// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.Climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstMotion;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepClimb extends Command {
  Drivetrain globalDrivetrain = RobotContainer.subDrivetrain;

  /** Creates a new PrepClimb. */
  public PrepClimb() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TOdo: replace nulls with proper values
    // globalDrivetrain.rotationalAlign(ConstField.TOWER_POSE,
    // ConstDrivetrain.TURN_SPEED);
    RobotContainer.motionInstance.setIntakePivotSpeed(ConstMotion.RETRACT_INTAKE_PIVOT_SPEED);
    RobotContainer.motionInstance.setClimberPosition(ConstMotion.EXTEND_CLIMBER);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.motionInstance.setClimberPosition(ConstMotion.STOP_CLIMBER);
    RobotContainer.motionInstance.setIntakePivotSpeed(ConstMotion.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
