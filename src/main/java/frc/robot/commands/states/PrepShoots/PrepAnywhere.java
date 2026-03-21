// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.PrepShoots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstPoseDrive;
import frc.robot.constants.ConstRotors;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Motion;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepAnywhere extends Command {

  public PrepAnywhere() {
    addRequirements(RobotContainer.stateMachineInstance);
  }

  @Override
  public void initialize() {
    RobotContainer.visionInstance.setVisionEnabled(true);
    RobotContainer.stateMachineInstance.setRobotState(RobotState.PREP_ANYWHERE);
    RobotContainer.drivetrainInstance.setIsManualRotationEnabled(false);
  }

  @Override
  public void execute() {
    Pose2d hubPose = RobotContainer.robotPose.getHub();
    Distance distanceToHub = RobotContainer.robotPose.getDistanceToHub();
    AngularVelocity targetFlyWheelSpeed = Rotors.getMappedFlywheelSpeed(distanceToHub);
    Angle targetHoodAngle = Motion.getMappedHoodAngle(distanceToHub);
    Angle targetDrivetrainRotation = RobotContainer.drivetrainInstance
        .snapToTarget(hubPose);

    RobotContainer.rotorsInstance.setFlywheelSpeed(targetFlyWheelSpeed);
    RobotContainer.motionInstance.setHoodAngle(targetHoodAngle);
    RobotContainer.drivetrainInstance.setDriveRotation(targetDrivetrainRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrainInstance.setIsManualRotationEnabled(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.rotorsInstance
        .areFlywheelsAtSpeed(ConstRotors.FLYWHEEL_TOLERANCE)
        && RobotContainer.motionInstance
            .isMotorAtPosition(ConstMotion.HOOD_TOLERANCE, RobotContainer.motionInstance.hood)
        && RobotContainer.drivetrainInstance
            .isAtDesiredRotation(ConstPoseDrive.PrepShootRotations.DRIVETRAIN_TOLERANCE);
  }
}
