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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
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
import frc.robot.constants.ConstAuto;
import frc.robot.constants.ConstDrivetrain;
import frc.robot.constants.ConstMotion;
import frc.robot.constants.ConstRotors;
import frc.robot.constants.ConstSystem;
import frc.robot.constants.ConstRumble;
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
import frc.robot.subsystems.Telemetry;

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
  Command TRY_PREP_CORNER = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_CORNER));
  Command TRY_PREP_DEPOT = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_DEPOT));
  Command TRY_PREP_TOWER = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_TOWER));
  Command TRY_PREP_HUB = Commands.deferredProxy(
      () -> stateMachineInstance.tryState(RobotState.PREP_HUB));
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

  private static AutoFactory autoFactory;

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
  public static Telemetry telemetryInstance = new Telemetry();
  private final Telemetry loggedTelemetryInstance = telemetryInstance;

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

  public final Trigger isOurShiftTrigger = new Trigger(
      () -> telemetryInstance.isHubActive());
  public final Trigger hubSwitchingTrigger = new Trigger(
      () -> telemetryInstance.hubsIsSwitching());
  public final Trigger climbingL1Trigger = new Trigger(
      () -> stateMachineInstance.getRobotState() == RobotState.CLIMBING_L1);
  public final Trigger climbingL2_L3Trigger = new Trigger(
      () -> stateMachineInstance.getRobotState() == RobotState.CLIMBING_L2_3);
  public final Trigger readyToShootTrigger = new Trigger(
      () -> rotorsInstance.areFlywheelsAtSpeed(ConstRotors.FLYWHEEL_TOLERANCE)
          && drivetrainInstance.isAtDesiredRotation(ConstDrivetrain.DRIVETRAIN_ROTATION_TOLERANCE)
          && motionInstance.isHoodAtPosition(ConstMotion.HOOD_TOLERANCE));

  public RobotContainer() {
    RobotController.setBrownoutVoltage(5.5);

    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    driverStateMachineInstance
        .setDefaultCommand(MANUAL);

    configDriverBindings();
    configOperatorBindings();
    configAutonomous();
    configFeedback();
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
        .whileTrue(TRY_PREP_CLIMB_L1)
        .onTrue(TRY_UNCLIMB_L1);
    // .onTrue(TRY_CLIMBING_L2_3);
    conDriver.btn_LeftTrigger
        .whileTrue(TRY_INTAKING)
        .onFalse(TRY_NONE);
    conDriver.btn_Back
        .onFalse(TRY_CLIMBING_L1);
    // .onTrue(TRY_DEFENSE)
    // .onFalse(TRY_NONE);
    conDriver.btn_RightBumper
        .onTrue(TRY_PREP_ANYWHERE);
    conDriver.btn_A
        .onTrue(TRY_PREP_TRENCH);
    conDriver.btn_West
        .onTrue(TRY_PREP_NEAUTRAL_TO_ALLIANCE)
        .onTrue(TRY_PREP_OPPONENT_TO_ALLIANCE);
    conDriver.btn_B
        .onTrue(TRY_PREP_CORNER);
    conDriver.btn_Y
        .onTrue(TRY_PREP_OPPONENT_TO_ALLIANCE);
    // .onTrue(TRY_PREP_HUB);
    // .onTrue(TRY_DEFENSE) rethink where to put defense
    // .onFalse(TRY_NONE);
    conDriver.btn_X
        .onTrue(TRY_PREP_TOWER);
    conDriver.btn_RightStick
        .onTrue(TRY_DEFENSE);
    conDriver.btn_North.whileTrue(new ResetPose());
  }

  public void configAutonomous() {
    autoFactory = new AutoFactory(
        drivetrainInstance::getPose, // A function that returns the current robot pose
        drivetrainInstance::resetPose, // A function that resets the current robot pose to the provided Pose2d
        drivetrainInstance::followTrajectory, // The drive subsystem trajectory follower
        true, // If alliance flipping should be enabled
        driverStateMachineInstance // The drive subsystem
    );

    Command PreloadOnly = Commands.sequence(
        ScoreOnly(ChoreoTraj.Hub_ShootPreload,
            TRY_PREP_ANYWHERE,
            ConstAuto.SHOOT_PRELOAD_TIMEOUT));

    Command PreloadWithClimb = Commands.sequence(PreloadOnly.asProxy(),
        Climb(ChoreoTraj.Preload_PrepClimb));

    Command PreloadDepot = Commands.sequence(
        CollectAndScore(ChoreoTraj.DSideBump_Depot,
            ChoreoTraj.Depot_DSidePrep,
            TRY_PREP_ANYWHERE,
            ConstAuto.INTAKE_DEPOT_TIMEOUT,
            ConstAuto.SHOOT_FROM_DEPOT_TIMEOUT));

    Command PreloadDepotWithClimb = Commands.sequence(
        PreloadDepot.asProxy(),
        Climb(ChoreoTraj.DSidePrep_PrepClimb));

    Command PreloadDepotWithOutpost = Commands.sequence(
        PreloadDepot.asProxy(),
        CollectAndScore(ChoreoTraj.DSidePrep_Outpost,
            ChoreoTraj.Outpost_OSidePrep,
            TRY_PREP_ANYWHERE,
            ConstAuto.INTAKE_OUTPOST_TIMEOUT,
            ConstAuto.SHOOT_FROM_OUTPOST_TIMEOUT));

    Command OutpostSideNeutral = Commands.sequence(
        TRY_INTAKING.asProxy().withTimeout(0.3), // Force intake down before moving and going under trench
        CollectAndScore(
            ChoreoTraj.OSideTrench_Neutral,
            ChoreoTraj.OSideNeutral_OSidePrep,
            TRY_PREP_ANYWHERE,
            ConstAuto.INTAKE_NEUTRAL_ZONE_TIMEOUT,
            ConstAuto.SHOOT_NEUTRAL_ZONE_TIMEOUT));

    Command DoubleOutpostSideNeutral = Commands.sequence(
        OutpostSideNeutral.asProxy(),
        runPath(ChoreoTraj.OSideShoot_OSideTrench).asProxy(),
        OutpostSideNeutral.asProxy());

    Command OutpostSideNeutralWithClimb = Commands.sequence(
        OutpostSideNeutral.asProxy(),
        Climb(ChoreoTraj.OSideShoot_PrepClimb));

    Command DepotSideNeutral = Commands.sequence(
        TRY_INTAKING.asProxy().withTimeout(0.3), // Force intake down before moving and going under trench
        CollectAndScore(
            ChoreoTraj.DSideTrench_Neutral,
            ChoreoTraj.DSideNeutral_DSidePrep,
            TRY_PREP_ANYWHERE,
            ConstAuto.INTAKE_NEUTRAL_ZONE_TIMEOUT,
            ConstAuto.SHOOT_NEUTRAL_ZONE_TIMEOUT));

    Command DoubleDepotSideNeutral = Commands.sequence(
        DepotSideNeutral.asProxy(),
        runPath(ChoreoTraj.DSideShoot_DSideTrench).asProxy(),
        DepotSideNeutral.asProxy());

    Command DepotSideNeutralWithClimb = Commands.sequence(
        DepotSideNeutral.asProxy(),
        Climb(ChoreoTraj.DSideBump_PrepClimb));

    Command DepotSideNeutralWithDepot = Commands.sequence(DepotSideNeutral.asProxy(),
        runPath(ChoreoTraj.DSidePrep_DSideBump).asProxy(),
        PreloadDepot.asProxy());

    Command PreloadOutpost = Commands.sequence(
        CollectAndScore(
            ChoreoTraj.OSideTrench_Outpost,
            ChoreoTraj.Outpost_OSidePrep,
            TRY_PREP_ANYWHERE,
            ConstAuto.INTAKE_OUTPOST_TIMEOUT,
            ConstAuto.SHOOT_FROM_OUTPOST_TIMEOUT));

    Command OutpostWithClimb = Commands.sequence(PreloadOutpost.asProxy(),
        Climb(ChoreoTraj.Outpost_PrepClimb));

    Command OutpostSideNeutralWithOutpost = Commands.sequence(
        OutpostSideNeutral.asProxy(),
        CollectAndScore(
            ChoreoTraj.OSideShoot_Outpost,
            ChoreoTraj.Outpost_OSidePrep,
            TRY_PREP_ANYWHERE,
            ConstAuto.INTAKE_OUTPOST_TIMEOUT,
            ConstAuto.SHOOT_FROM_OUTPOST_TIMEOUT));

    Command AutoPIDTuning = Commands.sequence(runPath(ChoreoTraj.AutoPIDTuning));

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("OutpostSideNeutralZone", OutpostSideNeutral);
    autoChooser.addOption("DoubleOutpostSideNeutral", DoubleOutpostSideNeutral);
    autoChooser.addOption("OutpostSideNeutralWithClimb", OutpostSideNeutralWithClimb);
    autoChooser.addOption("OutpostSideNeutralWithOutpost", OutpostSideNeutralWithOutpost);
    autoChooser.addOption("Outpost", PreloadOutpost);
    autoChooser.addOption("OutpostWithClimb", OutpostWithClimb);
    autoChooser.addOption("DepotSideNeutralZone", DepotSideNeutral);
    autoChooser.addOption("DoubleDepotSideNeutral", DoubleDepotSideNeutral);
    autoChooser.addOption("DepotSideNeutralWithClimb", DepotSideNeutralWithClimb);
    autoChooser.addOption("DepotSideNeutralWithDepot", DepotSideNeutralWithDepot);
    autoChooser.addOption("Depot", PreloadDepot);
    autoChooser.addOption("DepotWithClimb", PreloadDepotWithClimb);
    autoChooser.addOption("PreloadOnly", PreloadOnly);
    autoChooser.addOption("PreloadWithClimb", PreloadWithClimb);
    autoChooser.addOption("DepotOutpost", PreloadDepotWithOutpost);
    autoChooser.addOption("AutoPIDTuning", AutoPIDTuning);

    // make our entries name
    final Map<Command, ChoreoTraj> autoStartingPoses = Map.ofEntries(
        // Example
        Map.entry(PreloadOutpost, ChoreoTraj.OSideTrench_Outpost),
        Map.entry(OutpostWithClimb, ChoreoTraj.OSideTrench_Outpost),
        Map.entry(PreloadDepotWithOutpost, ChoreoTraj.DSideBump_Depot),
        Map.entry(PreloadDepotWithClimb, ChoreoTraj.DSideBump_Depot),
        Map.entry(PreloadOnly, ChoreoTraj.Hub_ShootPreload),
        Map.entry(PreloadWithClimb, ChoreoTraj.Hub_ShootPreload),
        Map.entry(DepotSideNeutral, ChoreoTraj.DSideTrench_Neutral),
        Map.entry(DoubleDepotSideNeutral, ChoreoTraj.DSideTrench_Neutral),
        Map.entry(PreloadDepot, ChoreoTraj.DSideBump_Depot),
        Map.entry(DepotSideNeutralWithClimb, ChoreoTraj.DSideTrench_Neutral),
        Map.entry(DepotSideNeutralWithDepot, ChoreoTraj.DSideTrench_Neutral),
        Map.entry(OutpostSideNeutral, ChoreoTraj.OSideTrench_Neutral),
        Map.entry(DoubleOutpostSideNeutral, ChoreoTraj.OSideTrench_Neutral),
        Map.entry(OutpostSideNeutralWithOutpost, ChoreoTraj.OSideTrench_Neutral),
        Map.entry(OutpostSideNeutralWithClimb, ChoreoTraj.OSideTrench_Neutral));

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

  Command CollectAndScore(ChoreoTraj collectPath, ChoreoTraj prepShootPath, Command try_prep_shoot, Time intakingTime,
      Time shootingTime) {
    return Commands.sequence(
        Commands.runOnce(() -> stateMachineInstance.setRobotState(RobotState.NONE)).asProxy(),
        runPath(collectPath).asProxy().deadlineFor(TRY_INTAKING.asProxy()),
        runPath(prepShootPath).asProxy().deadlineFor(TRY_INTAKING.asProxy()),
        TRY_NONE.asProxy(),
        try_prep_shoot.asProxy().withTimeout(ConstAuto.PREP_SHOOT_TIMEOUT),
        TRY_SHOOTING.asProxy().withTimeout(shootingTime),
        TRY_NONE.asProxy());
  }

  Command ScoreOnly(ChoreoTraj prepShootPath, Command try_prep_shoot, Time shootingTime) {
    return Commands.sequence(
        Commands.runOnce(() -> stateMachineInstance.setRobotState(RobotState.NONE)).asProxy(),
        runPath(prepShootPath).asProxy(),
        try_prep_shoot.asProxy().withTimeout(ConstAuto.PREP_SHOOT_TIMEOUT),
        TRY_SHOOTING.asProxy().withTimeout(shootingTime),
        TRY_NONE.asProxy());
  }

  Command Climb(ChoreoTraj startPath) {
    return Commands.sequence(
        Commands.runOnce(() -> stateMachineInstance.setRobotState(RobotState.NONE)).asProxy(),
        runPath(startPath).asProxy(),
        TRY_PREP_CLIMB_L1.asProxy().withTimeout(6),
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

  public void configFeedback() {
    readyToShootTrigger
        .whileTrue(
            Commands.run(() -> conDriver.setRumble(RumbleType.kLeftRumble,
                ConstRumble.READY_TO_SHOOT_RUMBLE)))
        .onFalse(Commands.runOnce(() -> conDriver.setRumble(RumbleType.kLeftRumble,
            ConstRumble.RUMBLE_OFF)));
    hubSwitchingTrigger
        .whileTrue(
            Commands.run(() -> conDriver.setRumble(RumbleType.kRightRumble,
                ConstRumble.SHIFT_CHANGE_RUMBLE)))
        .onFalse(Commands.runOnce(() -> conDriver.setRumble(RumbleType.kRightRumble,
            ConstRumble.RUMBLE_OFF)));
    // Add feedback bindings here if needed
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
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(false);
  }
}
