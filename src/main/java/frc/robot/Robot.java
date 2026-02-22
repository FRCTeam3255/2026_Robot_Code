// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstSystem;
import frc.robot.constants.ConstVision;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  static double autoTime = 140;
  static double shift1Time = 105;
  static double transitionShiftTime = 130;
  static double shift2Time = 80;
  static double shift3Time = 55;
  static double shift4Time = 30;
  static double endgameTime = 0;
  private RobotContainer m_robotContainer;
  private static final StringTopic selectedTabTopic = NetworkTableInstance.getDefault()
      .getStringTopic("/Elastic/SelectedTab");
  private static final StringPublisher selectedTabPublisher = selectedTabTopic
      .publish(PubSubOption.keepDuplicates(true));

  @Override
  public void robotInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    Epilogue.bind(this);
    m_robotContainer = new RobotContainer();

    // Set out log file to be in its own folder
    if (Robot.isSimulation()) {
      DataLogManager.start("logs");
    } else {
      DataLogManager.start();
    }
    // Log data that is being put to shuffleboard
    DataLogManager.logNetworkTables(true);
    // Log the DS data and joysticks
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    DriverStation.silenceJoystickConnectionWarning(ConstSystem.constControllers.SILENCE_JOYSTICK_WARNINGS);
    m_robotContainer.addVisionMeasurement().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    selectTab("Disabled");
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.DisabledThrottle);
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.DisabledThrottle);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, 1);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, 1);
  }

  @Override
  public void disabledPeriodic() {
    ConstField.ALLIANCE = DriverStation.getAlliance();
    SmartDashboard.putString("ALLIANCE", ConstField.ALLIANCE.toString());
  }

  @Override
  public void disabledExit() {
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.TeleopThrottle);
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.TeleopThrottle);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, 4);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, 4);
  }

  @Override
  public void autonomousInit() {
    selectTab("Autonomous");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  public static void selectTab(String tabName) {
    selectedTabPublisher.set(tabName);
  }

  @Override
  public void teleopInit() {
    selectTab("Teleoperated");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  public static double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public double getMatchPeriodTime() {
    double matchTime = getMatchTime();
    if (matchTime > autoTime) {
      return matchTime - autoTime;
    } else if (matchTime > transitionShiftTime) {
      return matchTime - transitionShiftTime;
    } else if (matchTime > shift1Time) {
      return matchTime - shift1Time;
    } else if (matchTime > shift2Time) {
      return matchTime - shift2Time;
    } else if (matchTime > shift3Time) {
      return matchTime - shift3Time;
    } else if (matchTime > shift4Time) {
      return matchTime - shift4Time;
    } else if (matchTime > endgameTime) {
      return matchTime;
    } else {
      return 0;
    }
  }

  public enum MatchPeriods {
    AUTO,
    TRANSITION_SHIFT,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME,
    POST_MATCH
  }

  public String getMatchPeriod() {
    if (getMatchTime() > autoTime) {
      return MatchPeriods.AUTO.toString();
    } else if (getMatchTime() > transitionShiftTime) {
      return MatchPeriods.TRANSITION_SHIFT.toString();
    } else if (getMatchTime() > shift1Time) {
      return MatchPeriods.SHIFT_1.toString();
    } else if (getMatchTime() > shift2Time) {
      return MatchPeriods.SHIFT_2.toString();
    } else if (getMatchTime() > shift3Time) {
      return MatchPeriods.SHIFT_3.toString();
    } else if (getMatchTime() > shift4Time) {
      return MatchPeriods.SHIFT_4.toString();
    } else if (getMatchTime() > endgameTime) {
      return MatchPeriods.ENDGAME.toString();
    } else {
      return MatchPeriods.POST_MATCH.toString();
    }
  }

  public static boolean hubsIsSwitching() {
    double matchTime = getMatchTime();
    if (matchTime < transitionShiftTime + 4 && matchTime > transitionShiftTime
        || matchTime < shift1Time + 4 && matchTime > shift1Time
        || matchTime < shift2Time + 4 && matchTime > shift2Time
        || matchTime < shift3Time + 4 && matchTime > shift3Time
        || matchTime < endgameTime + 4 && matchTime > shift4Time) {
      return true;
    }
    return false;
  }

  public static boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > transitionShiftTime) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > shift1Time) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > shift2Time) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > shift3Time) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > shift4Time) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }
}
