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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ConstField;
import frc.robot.constants.ConstSystem;
import frc.robot.constants.ConstVision;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private static final StringTopic selectedTabTopic = NetworkTableInstance.getDefault()
      .getStringTopic("/Elastic/SelectedTab");
  private static final StringPublisher selectedTabPublisher = selectedTabTopic
      .publish(PubSubOption.keepDuplicates(true));
  private boolean bothSubsystemsZeroed = false;
  private boolean hasAutonomousRun = false;

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

    bothSubsystemsZeroed = m_robotContainer.allZeroed();

    if (!hasAutonomousRun) {
      m_robotContainer.manualZeroSubsystems.schedule();
    }

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
    m_robotContainer.manualZeroSubsystems.cancel();
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.TeleopThrottle);
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.TeleopThrottle);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, 4);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, 4);
  }

  @Override
  public void autonomousInit() {
    selectTab("Autonomous");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    bothSubsystemsZeroed = m_robotContainer.allZeroed();

    if (bothSubsystemsZeroed && m_autonomousCommand != null) {
      Commands.deferredProxy(() -> m_autonomousCommand).schedule();
    } else if (m_autonomousCommand != null) {
      m_robotContainer.zeroSubsystems.andThen(Commands.deferredProxy(() -> m_autonomousCommand)).schedule();
      m_autonomousCommand.schedule();
    } else {
      m_robotContainer.zeroSubsystems.schedule();
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
    bothSubsystemsZeroed = m_robotContainer.allZeroed();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (!hasAutonomousRun || !bothSubsystemsZeroed) {
      m_robotContainer.zeroSubsystems.schedule();
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

  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public double getMatchPeriodTime() {
    double matchTime = getMatchTime();
    if (matchTime > 140) {
      return matchTime - 140;
    } else if (matchTime > 130) {
      return matchTime - 130;
    } else if (matchTime > 105) {
      return matchTime - 105;
    } else if (matchTime > 80) {
      return matchTime - 80;
    } else if (matchTime > 55) {
      return matchTime - 55;
    } else if (matchTime > 30) {
      return matchTime - 30;
    } else if (matchTime > 0) {
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
    if (getMatchTime() > 140) {
      return MatchPeriods.AUTO.toString();
    } else if (getMatchTime() > 130) {
      return MatchPeriods.TRANSITION_SHIFT.toString();
    } else if (getMatchTime() > 105) {
      return MatchPeriods.SHIFT_1.toString();
    } else if (getMatchTime() > 80) {
      return MatchPeriods.SHIFT_2.toString();
    } else if (getMatchTime() > 55) {
      return MatchPeriods.SHIFT_3.toString();
    } else if (getMatchTime() > 30) {
      return MatchPeriods.SHIFT_4.toString();
    } else if (getMatchTime() > 0) {
      return MatchPeriods.ENDGAME.toString();
    } else {
      return MatchPeriods.POST_MATCH.toString();
    }
  }

  public boolean isHubActive() {
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

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }
}
