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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.DeviceIDs.controllerIDs;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.ClimbingL1;
import frc.robot.commands.ClimbingL2_3;
import frc.robot.commands.ResetPose;
import frc.robot.commands.Shooting;
import frc.robot.commands.states.EjectingHopper;
import frc.robot.commands.states.Intaking;
import frc.robot.commands.states.ReverseShooter;
import frc.robot.commands.states.Unclimb;
import frc.robot.commands.states.PrepShoots.PrepAnywhere;
import frc.robot.commands.states.PrepShoots.PrepDepot;
import frc.robot.commands.states.PrepShoots.PrepNeutralToAlliance;
import frc.robot.commands.states.PrepShoots.PrepNonOutpost;
import frc.robot.commands.states.PrepShoots.PrepOpponentToAlliance;
import frc.robot.commands.states.PrepShoots.PrepOutpost;
import frc.robot.commands.states.PrepShoots.PrepTrench;
import frc.robot.constants.ChoreoTraj;
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
      () -> subStateMachine.tryState(RobotState.EJECTING_HOPPER));
  Command TRY_UNCLIMB_L1 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.UNCLIMB_L1));
  Command TRY_PREP_CLIMB_L1 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CLIMB_L1));
  Command TRY_CLIMBING_L1 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLIMBING_L1));
  Command TRY_CLIMBING_L2_3 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLIMBING_L2_3));
  Command TRY_INTAKING = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKING));
  Command TRY_SHOOTING = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SHOOTING));
  Command TRY_PREP_ANYWHERE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_ANYWHERE));
  Command TRY_PREP_TRENCH = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_TRENCH));
  Command TRY_PREP_OUPOST = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_OUTPOST));
  Command TRY_PREP_DEPOT = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_DEPOT));
  Command TRY_PREP_NON_OUTPOST = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_NON_OUTPOST));
  Command TRY_REVERSING_SHOOTER = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.REVERSING_SHOOTER));
  Command TRY_PREP_OPPONENT_TO_ALLIANCE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_OPPONENT_TO_ALLIANCE));
  Command TRY_PREP_NEAUTRAL_TO_ALLIANCE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_NEUTRAL_TO_ALLIANCE));
  Command TRY_NONE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.NONE));

  private AutoFactory autoFactory;

  private final SN_XboxController conDriver = new SN_XboxController(controllerIDs.DRIVER_USB);

  private static DigitalInput isPracticeBot = new DigitalInput(DeviceIDs.PRAC_BOT_DIO);

  public static Rotors rotorsInstance = new Rotors();
  private final Rotors loggedRotorsInstance = rotorsInstance;
  public static Motion motionInstance = new Motion();
  private final Motion loggedMotorsInstance = motionInstance;
  public static Drivetrain drivetrainInstance = new Drivetrain();
  private final Drivetrain loggedSubDrivetrain = drivetrainInstance;
  public static DriverStateMachine subDriverStateMachine = new DriverStateMachine(drivetrainInstance);
  private final DriverStateMachine loggedSubDriverStateMachine = subDriverStateMachine;
  public static StateMachine subStateMachine = new StateMachine(drivetrainInstance);
  private final StateMachine loggedSubStateMachine = subStateMachine;
  public static RobotPoses robotPose = new RobotPoses(drivetrainInstance);
  private final RobotPoses loggedRobotPose = robotPose;
  public static Vision subVision = new Vision();
  private final Vision loggedSubVision = subVision;

  Command MANUAL = new DeferredCommand(
      subDriverStateMachine.tryState(
          DriverStateMachine.DriverState.MANUAL,
          conDriver.axis_LeftY,
          conDriver.axis_LeftX,
          conDriver.axis_RightX,
          conDriver.axis_RightY,
          conDriver.btn_LeftBumper),
      Set.of(subDriverStateMachine));

  Command EXAMPLE_POSE_DRIVE = new DeferredCommand(
      subDriverStateMachine.tryState(
          DriverStateMachine.DriverState.EXAMPLE_POSE_DRIVE,
          conDriver.axis_LeftY,
          conDriver.axis_LeftX,
          conDriver.axis_RightX,
          conDriver.axis_RightY,
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
        .onTrue(TRY_PREP_CLIMB_L1)
        .onTrue(TRY_CLIMBING_L1)
        .onTrue(TRY_CLIMBING_L2_3);
    conDriver.btn_LeftTrigger
        .whileTrue(TRY_INTAKING)
        .onFalse(TRY_NONE);
    conDriver.btn_Back
        .onTrue(TRY_UNCLIMB_L1)
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
    autoFactory = new AutoFactory(
        drivetrainInstance::getPose, // A function that returns the current robot pose
        drivetrainInstance::resetPose, // A function that resets the current robot pose to the provided Pose2d
        drivetrainInstance::followTrajectory, // The drive subsystem trajectory follower
        true, // If alliance flipping should be enabled
        subDriverStateMachine // The drive subsystem
    );

    // make our entries name
    int shootingTime = 3;
    int intakingTime = 5;

    // Command PreloadOutpost = Commands.sequence(
    // ScoreAndCollect(ChoreoTraj.bump_to_hub,
    // ChoreoTraj.hub_to_outpost,
    // TRY_PREP_ANYWHERE,
    // shootingTime,
    // intakingTime),
    // ScoreOnly(ChoreoTraj.outpost_hub,
    // TRY_PREP_ANYWHERE,
    // shootingTime)
    // // runPath("bump_to_hub"),
    // // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy(),
    // // runPath("hub_to_outpost"),
    // // new PrepOutpost().withTimeout(4).asProxy(),
    // // runPath("outpost_hub")
    // // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy()
    // );

    Command PreloadOnly = Commands.sequence(
        ScoreOnly(ChoreoTraj.Reverse_From_Hub,
            TRY_PREP_ANYWHERE,
            shootingTime));

    Command PreloadDepot = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.Bump_HubLeft,
            ChoreoTraj.HubLeft_Depot,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.Depot_HubFront,
            TRY_PREP_ANYWHERE,
            shootingTime)
    // runPath("Bump_HubLeft").asProxy(),
    // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy(),
    // runPath("HubLeft_Depot").alongWith(new Intaking().withTimeout(.5)).asProxy(),
    // runPath("Depot_HubFront").asProxy()
    // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy()
    );

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
            shootingTime)
    // runPath("Reverse_From_Hub").asProxy(),
    // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy(),
    // runPath("HubFront_Outpost").withTimeout(5).asProxy(),
    // runPath("Outpost_HubFront").asProxy(),
    // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy(),
    // runPath("HubFront_Depot").alongWith(new
    // Intaking().withTimeout(.5)).asProxy(),
    // runPath("Depot_HubFront2").asProxy()
    // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy()
    );

    Command PreloadNeutralRight = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.OppBump_OppHub,
            ChoreoTraj.OppHub_OppNeutral,
            PreloadDepotOutpost,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.OppNeutral_OppHub,
            PreloadDepotOutpost,
            shootingTime)
    // runPath("OppBump_OppHub").asProxy(),
    // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy(),
    // runPath("OppHub_OppNeutral").alongWith(new
    // Intaking().withTimeout(5)).asProxy(),
    // runPath("OppNeutral_OppHub").alongWith(new
    // Intaking().withTimeout(.5)).asProxy()
    // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy()
    );

    Command PreloadNeutralLeft = Commands.sequence(
        ScoreAndCollect(ChoreoTraj.Bump_HubLeft,
            ChoreoTraj.HubLeft_Neutral,
            TRY_PREP_ANYWHERE,
            shootingTime,
            intakingTime),
        ScoreOnly(ChoreoTraj.Neutral_HubLeft,
            TRY_PREP_ANYWHERE,
            shootingTime)
    // runPath("Bump_HubLeft").asProxy(),
    // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy(),
    // runPath("HubLeft_Neutral").alongWith(new
    // Intaking().withTimeout(5)).asProxy(),
    // runPath("Neutral_HubLeft").asProxy()
    // new PrepAnywhere().alongWith(new Shooting().withTimeout(.5)).asProxy()
    );

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("PreloadDepot", PreloadDepot);
    autoChooser.addOption("PreloadDepotOutpost", PreloadDepotOutpost);
    // autoChooser.addOption("PreloadOutpost", PreloadOutpost);
    autoChooser.addOption("PreloadOnly", PreloadOnly);
    autoChooser.addOption("PreloadNeutralRight", PreloadNeutralRight);
    autoChooser.addOption("PreloadNeutralLeft", PreloadNeutralLeft);

    // make our entries name
    final Map<Command, ChoreoTraj> autoStartingPoses = Map.ofEntries(
        // Example
        // Map.entry(PreloadOutpost, "Trench_Outpost"),
        Map.entry(PreloadDepotOutpost, ChoreoTraj.Reverse_From_Hub),
        Map.entry(PreloadOnly, ChoreoTraj.Reverse_From_Hub),
        Map.entry(PreloadNeutralLeft, ChoreoTraj.Bump_HubLeft),
        Map.entry(PreloadDepot, ChoreoTraj.Bump_HubLeft),
        Map.entry(PreloadNeutralRight, ChoreoTraj.OppBump_OppHub));

    // enter which we want to do based on name
    autoChooser.onChange(selectedAuto ->

    {
      ChoreoTraj startingPose = autoStartingPoses.get(selectedAuto);
      // if there is a stating pose, reset to it
      if (startingPose != null) {
        autoFactory.resetOdometry(startingPose.name())
            .ignoringDisable(true) // Run even when disabled
            .schedule();
      }
    });

    // Example: Add autonomous routines to the chooser

    // Add more autonomous routines as needed, e.g.:
    // autoChooser.addOption("Score and Leave", runPath("ScoreAndLeave"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  Command ScoreAndCollect(ChoreoTraj startPath, ChoreoTraj endPath, Command try_prep_shoot, int shootingTime,
      int intakingTime) {
    return Commands.sequence(
        Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.NONE)).asProxy(),
        runPath(startPath).asProxy(),
        try_prep_shoot.asProxy().withTimeout(0.6),
        TRY_SHOOTING.asProxy().withTimeout(shootingTime),
        TRY_NONE.asProxy().withTimeout(0.05),
        runPath(endPath).asProxy().alongWith(TRY_INTAKING.asProxy().withTimeout(intakingTime)));
  }

  Command ScoreOnly(ChoreoTraj startPath, Command try_prep_shoot, int shootingTime) {
    return Commands.sequence(
        Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.NONE)).asProxy(),
        runPath(startPath).asProxy(),
        try_prep_shoot.asProxy().withTimeout(0.6),
        TRY_SHOOTING.asProxy().withTimeout(shootingTime));
  }

  Command Climb(ChoreoTraj startPath) {
    return Commands.sequence(
        Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.NONE)).asProxy(),
        runPath(startPath).asProxy(),
        TRY_PREP_CLIMB_L1.asProxy().withTimeout(0.5),
        TRY_CLIMBING_L1.asProxy().withTimeout(4));
  }

  public static boolean isPracticeBot() {
    return !isPracticeBot.get();
  }

  public String pathString = "";
  public Pose2d pathStartPose = new Pose2d();
  public Pose2d pathEndPose = new Pose2d();

  public Command runPath(ChoreoTraj path) {
    return autoFactory.trajectoryCmd(path.name()).asProxy()
        .alongWith(Commands.runOnce(() -> {
          pathString = path.name();
          pathStartPose = path.initialPoseBlue();
          pathEndPose = path.endPoseBlue();
          subDriverStateMachine.setDriverState(DriverState.CHOREO);
        }));
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
    return new AddVisionMeasurement(drivetrainInstance, subVision)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }
}
