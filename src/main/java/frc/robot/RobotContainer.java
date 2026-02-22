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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DeviceIDs.controllerIDs;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.ResetPose;
import frc.robot.constants.ChoreoTraj;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstRotors;
import frc.robot.constants.ConstSystem;
import frc.robot.constants.ConstSystem.constControllers;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.DriverStateMachine.DriverState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.RobotPoses;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Vision;

@Logged
public class RobotContainer {

  @NotLogged
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  // STATES
  Command TRY_EJECTING_HOPPER = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.EJECTING_HOPPER));
  Command TRY_UNCLIMB_L1 = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.UNCLIMB_L1));
  Command TRY_PREP_CLIMB_L1 = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_CLIMB_L1));
  Command TRY_CLIMBING_L1 = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.CLIMBING_L1));
  Command TRY_CLIMBING_L2_3 = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.CLIMBING_L2_3));
  Command TRY_INTAKING = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.INTAKING));
  Command TRY_SHOOTING = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.SHOOTING));
  Command TRY_PREP_ANYWHERE = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_ANYWHERE));
  Command TRY_PREP_TRENCH = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_TRENCH));
  Command TRY_PREP_OUPOST = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_OUTPOST));
  Command TRY_PREP_DEPOT = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_DEPOT));
  Command TRY_PREP_NON_OUTPOST = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_NON_OUTPOST));
  Command TRY_REVERSING_SHOOTER = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.REVERSING_SHOOTER));
  Command TRY_PREP_OPPONENT_TO_ALLIANCE = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_OPPONENT_TO_ALLIANCE));
  Command TRY_PREP_NEAUTRAL_TO_ALLIANCE = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_NEUTRAL_TO_ALLIANCE));
  Command TRY_NONE = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.NONE));
  Command TRY_DEFENSE = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.DEFENSE));

  private final SN_XboxController conDriver = new SN_XboxController(controllerIDs.DRIVER_USB);

  public static Rotors rotorsInstance = new Rotors();
  private final Rotors loggedRotorsInstance = rotorsInstance;
  public static Motion motionInstance = new Motion();
  private final Motion loggedMotionInstance = motionInstance;
  public static Drivetrain drivetrainInstance = new Drivetrain();
  private final Drivetrain loggedDrivetrainInstance = drivetrainInstance;
  public static RobotPoses robotPose = new RobotPoses();
  private final RobotPoses loggedRobotPose = robotPose;
  public static DriverStateMachine driverStateMachineInstance = new DriverStateMachine();
  private final DriverStateMachine loggedDriverStateMachineInstance = driverStateMachineInstance;
  public static StateMachine stateMachineInstance = new StateMachine();
  private final StateMachine loggedStateMachineInstance = stateMachineInstance;
  public static Vision visionInstance = new Vision();
  private final Vision loggedVisionInstance = visionInstance;

  private static AutoFactory autoFactory = new AutoFactory(
      drivetrainInstance::getPose, // A function that returns the current robot pose
      drivetrainInstance::resetPose, // A function that resets the current robot pose to the provided Pose2d
      drivetrainInstance::followTrajectory, // The drive subsystem trajectory follower
      true, // If alliance flipping should be enabled
      driverStateMachineInstance // The drive subsystem
  );
  Command MANUAL = new DeferredCommand(
      driverStateMachineInstance.tryState(
          DriverStateMachine.DriverState.MANUAL,
          conDriver.axis_LeftY,
          conDriver.axis_LeftX,
          conDriver.axis_RightX,
          conDriver.axis_RightY,
          conDriver.btn_LeftBumper),
      Set.of(driverStateMachineInstance));

  Command EXAMPLE_POSE_DRIVE = new DeferredCommand(
      driverStateMachineInstance.tryState(
          DriverStateMachine.DriverState.EXAMPLE_POSE_DRIVE,
          conDriver.axis_LeftY,
          conDriver.axis_LeftX,
          conDriver.axis_RightX,
          conDriver.axis_RightY,
          conDriver.btn_RightBumper),
      Set.of(driverStateMachineInstance));

  public RobotContainer() {
    RobotController.setBrownoutVoltage(5.5);

    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    driverStateMachineInstance
        .setDefaultCommand(MANUAL);

    configDriverBindings();
    configOperatorBindings();
    configAutonomous();
    // subDrivetrain.resetModulesToAbsolute();
  }

  public final Trigger climbingL1Trigger = new Trigger(
      () -> stateMachineInstance.getRobotState() == RobotState.CLIMBING_L1);
  public final Trigger climbingL2_L3Trigger = new Trigger(
      () -> stateMachineInstance.getRobotState() == RobotState.CLIMBING_L2_3);
  public final Trigger readyToShootTrigger = new Trigger(
      () -> rotorsInstance.areFlywheelsAtSpeed(ConstRotors.FLYWHEEL_TOLERANCE)
          && drivetrainInstance.isAtDesiredRotation(ConstDrivetrain.DRIVETRAIN_ROTATION_TOLERANCE)
          && motionInstance.isHoodAtPosition(ConstMotion.HOOD_TOLERANCE));

  private void configDriverBindings() {
    conDriver.btn_South
        .whileTrue(TRY_EJECTING_HOPPER)
        .onFalse(TRY_NONE);
    conDriver.btn_RightTrigger
        .whileTrue(TRY_SHOOTING)
        .onFalse(TRY_NONE);
    conDriver.btn_East
        .whileTrue(TRY_REVERSING_SHOOTER)
        .onFalse(TRY_NONE);
    conDriver.btn_Start
        .whileTrue(TRY_PREP_CLIMB_L1)
        .onFalse(TRY_CLIMBING_L1)
        .onTrue(TRY_CLIMBING_L2_3);
    conDriver.btn_LeftTrigger
        .whileTrue(TRY_INTAKING)
        .onFalse(TRY_NONE);
    conDriver.btn_Back
        .onTrue(TRY_UNCLIMB_L1)
        .onTrue(TRY_DEFENSE)
        .onFalse(TRY_NONE);
    conDriver.btn_RightBumper
        .onTrue(TRY_PREP_ANYWHERE);
    conDriver.btn_A
        .onTrue(TRY_PREP_DEPOT);
    conDriver.btn_West
        .onTrue(TRY_PREP_NEAUTRAL_TO_ALLIANCE)
        .onTrue(TRY_PREP_OPPONENT_TO_ALLIANCE);
    conDriver.btn_B
        .onTrue(TRY_PREP_OUPOST);
    conDriver.btn_Y
        .onTrue(TRY_PREP_TRENCH);
    conDriver.btn_X
        .onTrue(TRY_PREP_NON_OUTPOST);
    conDriver.btn_North.onTrue(new ResetPose());
  }

  public void configAutonomous() {

    // make our entries name
    int shootingTime = 5;
    int intakingTime = 7;

    Command PreloadOnly = Commands.sequence(
        ScoreOnly(ChoreoTraj.Reverse_From_Hub,
            TRY_PREP_ANYWHERE,
            shootingTime));

    Command PreloadWithClimb = Commands.sequence(
        ScoreOnly(ChoreoTraj.Reverse_From_Hub,
            TRY_PREP_ANYWHERE,
            shootingTime),
        Climb(ChoreoTraj.Reverse_From_Hub));

    Command PreloadDepot = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.Bump_HubLeft,
            ChoreoTraj.HubLeft_Depot,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.Depot_HubFront,
            TRY_PREP_ANYWHERE,
            shootingTime));

    Command PreloadDepotWithClimb = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.Bump_HubLeft,
            ChoreoTraj.HubLeft_Depot,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.Depot_HubFront,
            TRY_PREP_ANYWHERE,
            shootingTime),
        Climb(ChoreoTraj.Depot_HubFront));

    Command PreloadDepotOutpost = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.Reverse_From_Hub,
            ChoreoTraj.HubFront_Outpost,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreAndCollect(ChoreoTraj.Outpost_HubFront,
            ChoreoTraj.HubFront_Depot,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.Depot_HubFront2,
            TRY_PREP_ANYWHERE,
            shootingTime));

    Command PreloadDepotOutpostWithClimb = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.Reverse_From_Hub,
            ChoreoTraj.HubFront_Outpost,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreAndCollect(ChoreoTraj.Outpost_HubFront,
            ChoreoTraj.HubFront_Depot,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.Depot_HubFront2,
            TRY_PREP_ANYWHERE,
            shootingTime),
        Climb(ChoreoTraj.Depot_HubFront2));

    Command PreloadNeutralRight = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.OppBump_OppHub,
            ChoreoTraj.OppHub_OppNeutral,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.OppNeutral_OppHub,
            TRY_PREP_ANYWHERE,
            shootingTime));

    Command PreloadNeutralRightWithClimb = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.OppBump_OppHub,
            ChoreoTraj.OppHub_OppNeutral,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.OppNeutral_OppHub,
            TRY_PREP_ANYWHERE,
            shootingTime),
        Climb(ChoreoTraj.OppNeutral_OppHub));

    Command PreloadNeutralLeft = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.Bump_HubLeft,
            ChoreoTraj.HubLeft_Neutral,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.Neutral_HubLeft,
            TRY_PREP_ANYWHERE,
            shootingTime));

    Command PreloadNeutralLeftWithClimb = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.Bump_HubLeft,
            ChoreoTraj.HubLeft_Neutral,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.Neutral_HubLeft,
            TRY_PREP_ANYWHERE,
            shootingTime),
        Climb(ChoreoTraj.Neutral_HubLeft));

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("PreloadDepot", PreloadDepot);
    autoChooser.addOption("PreloadDepotOutpost", PreloadDepotOutpost);
    // autoChooser.addOption("PreloadOutpost", PreloadOutpost);
    autoChooser.addOption("PreloadOnly", PreloadOnly);
    autoChooser.addOption("PreloadNeutralRight", PreloadNeutralRight);
    autoChooser.addOption("PreloadNeutralRightWithClimb", PreloadNeutralRightWithClimb);
    autoChooser.addOption("PreloadNeutralLeft", PreloadNeutralLeft);
    autoChooser.addOption("PreloadNeutralLeftWithClimb", PreloadNeutralLeftWithClimb);
    autoChooser.addOption("PreloadDepotWithClimb", PreloadDepotWithClimb);
    autoChooser.addOption("PreloadDepotOutpostWithClimb", PreloadDepotOutpostWithClimb);
    autoChooser.addOption("PreloadWithClimb", PreloadWithClimb);

    // make our entries name
    final Map<Command, ChoreoTraj> autoStartingPoses = Map.ofEntries(
        // Example
        // Map.entry(PreloadOutpost, "Trench_Outpost"),
        Map.entry(PreloadOnly, ChoreoTraj.Reverse_From_Hub),
        Map.entry(PreloadWithClimb, ChoreoTraj.Reverse_From_Hub),
        Map.entry(PreloadDepotOutpost, ChoreoTraj.Reverse_From_Hub),
        Map.entry(PreloadDepotOutpostWithClimb, ChoreoTraj.Reverse_From_Hub),
        Map.entry(PreloadDepot, ChoreoTraj.Bump_HubLeft),
        Map.entry(PreloadDepotWithClimb, ChoreoTraj.Reverse_From_Hub),
        Map.entry(PreloadNeutralLeft, ChoreoTraj.Bump_HubLeft),
        Map.entry(PreloadNeutralLeftWithClimb, ChoreoTraj.Bump_HubLeft),
        Map.entry(PreloadNeutralRight, ChoreoTraj.OppBump_OppHub),
        Map.entry(PreloadNeutralRightWithClimb, ChoreoTraj.OppBump_OppHub));

    // enter which we want to do based on name
    autoChooser.onChange(selectedAuto ->

    {
      ChoreoTraj startingPose = autoStartingPoses.get(selectedAuto);
      // if there is a starting pose, reset to it
      if (startingPose != null) {
        autoFactory.resetOdometry(startingPose.name())
            .ignoringDisable(true) // Run even when disabled
            .schedule();
      }
    });

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  Command ScoreAndCollect(ChoreoTraj startPath, ChoreoTraj endPath, Command try_prep_shoot, int shootingTime,
      int intakingTime) {
    return Commands.sequence(
        Commands.runOnce(() -> stateMachineInstance.setRobotState(RobotState.NONE)).asProxy(),
        runPath(startPath).asProxy(),
        try_prep_shoot.asProxy().withTimeout(1.5),
        TRY_SHOOTING.asProxy().withTimeout(shootingTime),
        TRY_NONE.asProxy().withTimeout(0.05),
        runPath(endPath).asProxy().alongWith(TRY_INTAKING.asProxy().withTimeout(intakingTime)));
  }

  Command ScoreOnly(ChoreoTraj startPath, Command try_prep_shoot, int shootingTime) {
    return Commands.sequence(
        Commands.runOnce(() -> stateMachineInstance.setRobotState(RobotState.NONE)).asProxy(),
        runPath(startPath).asProxy(),
        try_prep_shoot.asProxy().withTimeout(0.6),
        TRY_SHOOTING.asProxy().withTimeout(shootingTime));
  }

  Command Climb(ChoreoTraj startPath) {
    return Commands.sequence(
        Commands.runOnce(() -> stateMachineInstance.setRobotState(RobotState.NONE)).asProxy(),
        TRY_PREP_CLIMB_L1.asProxy().withTimeout(0.5),
        TRY_CLIMBING_L1.asProxy().withTimeout(4));
  }

  public static boolean isPracticeBot() {
    return RobotController.getSerialNumber().equals(ConstSystem.PRACTICE_BOT_RIO);
  }

  public static String pathString = "";
  public static Pose2d pathStartPose = new Pose2d();
  public static Pose2d pathEndPose = new Pose2d();

  public static Command runPath(ChoreoTraj path) {
    return autoFactory.trajectoryCmd(path.name()).asProxy()
        .alongWith(Commands.runOnce(() -> {
          pathString = path.name();
          pathStartPose = path.initialPoseBlue();
          pathEndPose = path.endPoseBlue();
          driverStateMachineInstance.setDriverState(DriverState.CHOREO);
        }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configOperatorBindings() {
    // Add operator bindings here if needed
  }

  public RobotState getRobotState() {
    return stateMachineInstance.getRobotState();
  }

  public String robotStateToString() {
    return stateMachineInstance.getRobotState().toString();
  }

  public String driverStateToString() {
    return driverStateMachineInstance.getDriverState().toString();
  }

  public Command addVisionMeasurement() {
    return new AddVisionMeasurement()
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }
}
