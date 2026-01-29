// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.PrepShoots;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstRotors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepAnywhere extends Command {
  public PrepAnywhere() {
  }

  @Override
  public void initialize() {
    RobotContainer.drivetrainInstance.setIsManualRotationEnabled(false);
    System.out.println("enters prep anywhere");
  }

  @Override
  public void execute() {
    // Angle globalHoodAngle = ConstMotion.ShooterHoodTable
    // .getHoodAngle(Meters.of(ConstField.HUB_POSE.getTranslation().getNorm()));
    Angle globalDrivetrainRotation = RobotContainer.drivetrainInstance.getToTarget(ConstField.HUB_POSE);

    System.out.print(globalDrivetrainRotation);
    RobotContainer.rotorsInstance.setFlywheelSpeed(ConstRotors.FLYWHEEL_ANYWHERE_SPEED);
    // RobotContainer.motionInstance.setHoodAngle(globalHoodAngle);
    RobotContainer.drivetrainInstance.setDriveRotation(globalDrivetrainRotation);
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
