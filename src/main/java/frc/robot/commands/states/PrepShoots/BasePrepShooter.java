// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.PrepShoots;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstRotors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasePrepShooter extends Command {
  /** Creates a new BasePrepShooter. */
  AngularVelocity globalFlyWheelSpeed;
  Angle globalHoodAngle;
  Angle globalDrivetrainAngle;

  public BasePrepShooter(AngularVelocity flyWheelSpeed, Angle hoodAngle, Angle drivetrainAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalFlyWheelSpeed = flyWheelSpeed;
    globalHoodAngle = hoodAngle;
    globalDrivetrainAngle = drivetrainAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.rotorsInstance.setFlywheelSpeed(globalFlyWheelSpeed);
    RobotContainer.motionInstance.setHoodAngle(globalHoodAngle);
    RobotContainer.drivetrainInstance.setDriveRotation(globalDrivetrainAngle);
    RobotContainer.drivetrainInstance.setIsManualRotationEnabled(false);
    System.out.println("enters base prep shoot");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.rotorsInstance.setFlywheelPercentOutput(ConstRotors.STOP);
    RobotContainer.motionInstance.setHoodAngle(ConstMotion.HOOD_NONE_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
