// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConstRotors;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectingHopper extends Command {
  /** Creates a new None. */

  public EjectingHopper() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.rotorsInstance.setSerializerRollersSpeed(ConstRotors.REVERSE_SERIALIZER_ROLLERS_SPEED);
    RobotContainer.rotorsInstance.setIntakeRollersSpeed(ConstRotors.REVERSE_INTAKE_ROLLER_SPEED);
    RobotContainer.rotorsInstance.setSerializerVFunnelSpeed(ConstRotors.REVERSE_SERIALIZER_V_FUNNEL_SPEED);
    RobotContainer.drivetrainInstance.resetDriveRotationBool();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.rotorsInstance.setSerializerRollersSpeed(0);
    RobotContainer.rotorsInstance.setIntakeRollersSpeed(0);
    RobotContainer.rotorsInstance.setSerializerVFunnelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
