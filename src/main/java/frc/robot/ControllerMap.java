// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;
import com.frcteam3255.joystick.SN_XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs.controllerIDs;
import frc.robot.constants.ConstSystem.constControllers;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.DriverStateMachine.DriverState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

public class ControllerMap extends SubsystemBase {
  private static final SN_XboxController conDriver = new SN_XboxController(controllerIDs.DRIVER_USB);
  private static final SN_XboxController conOperator = new SN_XboxController(controllerIDs.OPERATOR_USB);

  static Command EXAMPLE_POSE_DRIVE = new DeferredCommand(
      RobotContainer.subDriverStateMachine.tryState(
          DriverStateMachine.DriverState.EXAMPLE_POSE_DRIVE,
          conDriver.axis_LeftY,
          conDriver.axis_LeftX,
          conDriver.axis_RightX,
          conDriver.btn_RightBumper),
      Set.of(RobotContainer.subDriverStateMachine));

  Command TRY_NONE = Commands.deferredProxy(
      () -> RobotContainer.subStateMachine.tryState(RobotState.NONE));

  Command MANUAL = new DeferredCommand(
      RobotContainer.subDriverStateMachine.tryState(
          DriverStateMachine.DriverState.MANUAL,
          conDriver.axis_LeftY,
          conDriver.axis_LeftX,
          conDriver.axis_RightX,
          conDriver.btn_RightBumper),
      Set.of(RobotContainer.subDriverStateMachine));

  /** Creates a new ControllerMap. */
  public ControllerMap() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    RobotContainer.subDriverStateMachine
        .setDefaultCommand(MANUAL);

  }

  public static void configDriverBindings() {
    // remember conDriver.(what button).on/while,True/False(Commands.runOnce(() ->
    // (what you want to do)));

    conDriver.btn_Back
        .onTrue(Commands.runOnce(() -> RobotContainer.subDrivetrain.resetPose(new Pose2d(0, 0, new Rotation2d()))));

    // Example Pose Drive
    conDriver.btn_X
        .whileTrue(EXAMPLE_POSE_DRIVE)
        .onFalse(Commands.runOnce(() -> RobotContainer.subDriverStateMachine.setDriverState(DriverState.MANUAL)));

  }

  public static void configOperatorBindings() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
