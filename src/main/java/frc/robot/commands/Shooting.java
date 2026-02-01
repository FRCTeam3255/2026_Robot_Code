// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstRotors;
import frc.robot.subsystems.StateMachine;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shooting extends Command {
  StateMachine globalStateMachine;

  /** Creates a new Shooting. */
  public Shooting(StateMachine globalStateMachine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.globalStateMachine = globalStateMachine;
    addRequirements(RobotContainer.subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(StateMachine.RobotState.SHOOTING);
    RobotContainer.rotorsInstance.setSerializerRollersSpeed(ConstRotors.SERIALIZER_ROLLERS_SPEED);
    RobotContainer.rotorsInstance.setSerializerVFunnelSpeed(ConstRotors.SERIALIZER_V_FUNNEL_SPEED);
    RobotContainer.rotorsInstance.setShooterTransferSpeed(ConstRotors.SHOOTER_TRANSFER_SPEED);
    RobotContainer.rotorsInstance.setIntakeRollersSpeed(ConstRotors.INTAKE_ROLLER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drivetrainInstance.xBrake();
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
