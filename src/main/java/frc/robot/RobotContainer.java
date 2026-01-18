// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Set;

import com.frcteam3255.joystick.SN_XboxController;

import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.DeviceIDs.controllerIDs;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.constants.ConstSystem.constControllers;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.DriverStateMachine.DriverState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RobotPoses;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Vision;

@Logged
public class RobotContainer {
  @NotLogged
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  private AutoFactory autoFactory;

  private final SN_XboxController conDriver = new SN_XboxController(controllerIDs.DRIVER_USB);

  public final static Rotors rotorsInstance = new Rotors();
  public final static Drivetrain subDrivetrain = new Drivetrain();
  public final static DriverStateMachine subDriverStateMachine = new DriverStateMachine(subDrivetrain);
  public final static StateMachine subStateMachine = new StateMachine(subDrivetrain);
  public final RobotPoses robotPose = new RobotPoses(subDrivetrain);
  public final static Vision subVision = new Vision();

  Command TRY_NONE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.NONE));

  Command MANUAL = new DeferredCommand(
      subDriverStateMachine.tryState(
          DriverStateMachine.DriverState.MANUAL,
          conDriver.axis_LeftY,
          conDriver.axis_LeftX,
          conDriver.axis_RightX,
          conDriver.btn_RightBumper),
      Set.of(subDriverStateMachine));

  Command EXAMPLE_POSE_DRIVE = new DeferredCommand(
      subDriverStateMachine.tryState(
          DriverStateMachine.DriverState.EXAMPLE_POSE_DRIVE,
          conDriver.axis_LeftY,
          conDriver.axis_LeftX,
          conDriver.axis_RightX,
          conDriver.btn_RightBumper),
      Set.of(subDriverStateMachine));

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDriverStateMachine
        .setDefaultCommand(MANUAL);

    configDriverBindings();
    configOperatorBindings();
    configAutonomous();

    // subDrivetrain.resetModulesToAbsolute();
  }

  private void configDriverBindings() {
    // conDriver.btn_B.onTrue(Commands.runOnce(() ->
    // subDrivetrain.resetModulesToAbsolute()));
    conDriver.btn_Back
        .onTrue(Commands.runOnce(() -> subDrivetrain.resetPose(new Pose2d(0, 0, new Rotation2d()))));

    // Example Pose Drive
    conDriver.btn_X
        .whileTrue(EXAMPLE_POSE_DRIVE)
        .onFalse(Commands.runOnce(() -> subDriverStateMachine.setDriverState(DriverState.MANUAL)));
  }

  public void configAutonomous() {
    autoFactory = new AutoFactory(
        subDrivetrain::getPose, // A function that returns the current robot pose
        subDrivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
        subDrivetrain::followTrajectory, // The drive subsystem trajectory follower
        true, // If alliance flipping should be enabled
        subDriverStateMachine // The drive subsystem
    );

    // make our entries name
    final Map<Command, String> autoStartingPoses = Map.ofEntries(
    // Example
    // Map.entry(autoCommand, "choreoStartingPath"),
    );
    // enter which we want to do based on name
    autoChooser.onChange(selectedAuto -> {
      String startingPose = autoStartingPoses.get(selectedAuto);
      // if there is a stating pose, reset to it
      if (startingPose != null) {
        autoFactory.resetOdometry(startingPose)
            .ignoringDisable(true) // Run even when disabled
            .schedule();
      }
    });

    // Example: Add autonomous routines to the chooser
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Example Path", runPath("ExamplePath"));
    // Add more autonomous routines as needed, e.g.:
    // autoChooser.addOption("Score and Leave", runPath("ScoreAndLeave"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command runPath(String pathName) {
    return autoFactory.trajectoryCmd(pathName).asProxy()
        .alongWith(Commands.runOnce(() -> subDriverStateMachine.setDriverState(DriverState.CHOREO)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configOperatorBindings() {
    // Add operator bindings here if needed
  }

  public RobotState getRobotState() {
    return subStateMachine.getRobotState();
  }

  public Command addVisionMeasurement() {
    return new AddVisionMeasurement(subDrivetrain, subVision)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }
}
