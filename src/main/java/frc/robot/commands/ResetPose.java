// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstField;

public class ResetPose extends Command {

  public ResetPose() {
    addRequirements(RobotContainer.subDriverStateMachine);
  }

  @Override
  public void initialize() {
    RobotContainer.drivetrainInstance.resetPose(ConstField.FieldElements.RESET_POSE);
    RobotContainer.drivetrainInstance.getPigeon2()
        .setYaw(ConstField.FieldElements.RESET_POSE.getRotation().getMeasure());
    RobotContainer.drivetrainInstance.setDriveRotation(ConstField.FieldElements.RESET_POSE.getRotation().getMeasure());
    System.out.println(ConstField.FieldElements.RESET_POSE.getRotation().getMeasure().in(Units.Degrees));
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
