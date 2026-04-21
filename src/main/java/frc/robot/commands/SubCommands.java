// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstMotion;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine.RobotState;

/** Add your docs here. */
public class SubCommands {

  public static void aim(boolean aimDT) {
    Pose2d hubPose = RobotContainer.robotPose.getHub();
    Distance distanceToHub = RobotContainer.robotPose.getDistanceToHub();
    AngularVelocity targetFlyWheelSpeed = Rotors.getMappedFlywheelSpeed(distanceToHub);
    Angle targetHoodAngle = Motion.getMappedHoodAngle(distanceToHub);
    Angle targetDrivetrainRotation = RobotContainer.drivetrainInstance
        .snapToTarget(hubPose);

    RobotContainer.rotorsInstance.setFlywheelSpeed(targetFlyWheelSpeed);
    RobotContainer.motionInstance.setHoodAngle(targetHoodAngle);
    if (aimDT) {
      RobotContainer.drivetrainInstance.setDriveRotation(targetDrivetrainRotation);
    }
  }
}
