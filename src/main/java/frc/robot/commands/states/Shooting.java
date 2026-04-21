// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.SubCommands;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstRotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shooting extends Command {
  private final Timer deployIntakeTimer = new Timer();
  private RobotState previousState;

  /** Creates a new Shooting. */
  public Shooting() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(RobotContainer.stateMachineInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousState = RobotContainer.stateMachineInstance.getRobotState();
    RobotContainer.stateMachineInstance.setRobotState(StateMachine.RobotState.SHOOTING);
    RobotContainer.rotorsInstance.setSerializerRollersSpeed(ConstRotors.SERIALIZER_ROLLERS_SPEED);
    RobotContainer.rotorsInstance.setShooterTransferSpeed(ConstRotors.SHOOTER_TRANSFER_SPEED);
    RobotContainer.rotorsInstance.setIntakeRollersSpeed(ConstRotors.INTAKE_ROLLER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (previousState == RobotState.PREP_ANYWHERE
        && RobotContainer.visionInstance.seesTags()
        && !RobotContainer.drivetrainInstance.isAtDesiredRotation(ConstDrivetrain.DRIVETRAIN_ROTATION_TOLERANCE)) {
      SubCommands
          .aim(true);
    } else if (previousState == RobotState.PREP_ANYWHERE
        && RobotContainer.visionInstance.seesTags()
        && !RobotContainer.motionInstance.isHoodAtPosition(ConstMotion.HOOD_TOLERANCE)) {
      SubCommands.aim(false);
      RobotContainer.drivetrainInstance.xBrake();
    } else if (RobotContainer.drivetrainInstance.isXbreakAllowed()) {
      RobotContainer.drivetrainInstance.xBrake();
    }

    if (!RobotContainer.drivetrainInstance.isXbreakAllowed()) {
      RobotContainer.motionInstance.setIntakePivotAngle(ConstMotion.DEPLOY_INTAKE_PIVOT_ANGLE);
      deployIntakeTimer.reset();
      return;
    }
    deployIntakeTimer.start();
    if (!RobotContainer.motionInstance.isIntakePivotAtPosition(ConstMotion.LIFT_INTAKE_SHOOTING_ANGLE,
        ConstMotion.INTAKE_PIVOT_ANGLE_TOLERANCE)
        && deployIntakeTimer.hasElapsed(ConstMotion.LIFT_INTAKE_INTERVAL_TIME)) {
      RobotContainer.motionInstance.setIntakePivotAngle(ConstMotion.LIFT_INTAKE_SHOOTING_ANGLE);
      deployIntakeTimer.stop();
      deployIntakeTimer.reset();
      deployIntakeTimer.start();
    } else if (deployIntakeTimer.hasElapsed(ConstMotion.LIFT_INTAKE_INTERVAL_TIME)) {
      RobotContainer.motionInstance.setIntakePivotAngle(ConstMotion.DEPLOY_INTAKE_PIVOT_ANGLE);
      deployIntakeTimer.stop();
      deployIntakeTimer.reset();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    deployIntakeTimer.stop();
    RobotContainer.motionInstance.setIntakePivotAngle(ConstMotion.DEPLOY_INTAKE_PIVOT_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
