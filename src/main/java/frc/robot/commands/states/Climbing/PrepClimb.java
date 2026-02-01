// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.Climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstMotion;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.StateMachine;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepClimb extends Command {
  Drivetrain globalDrivetrain = RobotContainer.drivetrainInstance;
  StateMachine globalStateMachine;

  /** Creates a new PrepClimb. */
  public PrepClimb(StateMachine globalStateMachine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.globalStateMachine = globalStateMachine;
    addRequirements(RobotContainer.subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TOdo: replace nulls with proper values
    // globalDrivetrain.rotationalAlign(ConstField.TOWER_POSE,
    // ConstDrivetrain.TURN_SPEED);
    globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CLIMB_L1);
    RobotContainer.motionInstance.setClimberPosition(ConstMotion.EXTEND_CLIMBER);
    RobotContainer.motionInstance.setIntakePivotAngle(ConstMotion.RETRACT_INTAKE_PIVOT_ANGLE);

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
    return RobotContainer.motionInstance.isAtPosition(ConstMotion.EXTEND_CLIMBER, ConstMotion.CLIMBER_TOLERANCE);
  }
}
