// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.PrepShoots;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstField.Pose2dAllianceSet;
import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstPoseDrive;
import frc.robot.constants.ConstRotors;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasePrepShooter extends Command {
  /** Creates a new BasePrepShooter. */
  AngularVelocity globalFlyWheelSpeed;
  Angle globalHoodAngle;
  RobotState globalState;

  public BasePrepShooter(AngularVelocity flyWheelSpeed, Angle hoodAngle,
      RobotState state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.stateMachineInstance);
    globalFlyWheelSpeed = flyWheelSpeed;
    globalHoodAngle = hoodAngle;
    globalState = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.rotorsInstance.setFlywheelSpeed(globalFlyWheelSpeed);
    RobotContainer.motionInstance.setHoodAngle(globalHoodAngle);
    RobotContainer.drivetrainInstance.setIsManualRotationEnabled(false);
    RobotContainer.stateMachineInstance.setRobotState(globalState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrainInstance.setIsManualRotationEnabled(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.motionInstance
        .isHoodAtPosition(ConstMotion.HOOD_TOLERANCE)
        && RobotContainer.rotorsInstance
            .areFlywheelsAtSpeed(ConstRotors.FLYWHEEL_TOLERANCE);
  }
}
