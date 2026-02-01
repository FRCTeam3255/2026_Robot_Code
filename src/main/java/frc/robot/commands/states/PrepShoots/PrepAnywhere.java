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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepAnywhere extends Command {

  public PrepAnywhere() {
    addRequirements(RobotContainer.subStateMachine);
  }

  @Override
  public void initialize() {
    RobotContainer.drivetrainInstance.setIsManualRotationEnabled(false);
  }

  @Override
  public void execute() {
    Angle targetHoodAngle = RobotContainer.motionInstance.getToTargetHoodAngle();

    Angle targetDrivetrainRotation = RobotContainer.drivetrainInstance
        .snapToTarget(RobotContainer.robotPose.getHub());

    AngularVelocity targetFlyWheelSpeed = RobotContainer.rotorsInstance
        .getToTargetFlywheelSpeed();

    RobotContainer.rotorsInstance.setFlywheelSpeed(targetFlyWheelSpeed);
    RobotContainer.motionInstance.setHoodAngle(targetHoodAngle);
    RobotContainer.drivetrainInstance.setDriveRotation(targetDrivetrainRotation);
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
