// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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

  private RobotContainer m_robotContainer;

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

  @Override
  public void teleopInit() {
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

  public String getMatchPeriod() {
    if (getMatchTime() > 140) {
      return "Auto";
    } else if (getMatchTime() > 130) {
      return "Transition Shift";
    } else if (getMatchTime() > 105) {
      return "Shift 1";
    } else if (getMatchTime() > 80) {
      return "Shift 2";
    } else if (getMatchTime() > 55) {
      return "Shift 3";
    } else if (getMatchTime() > 30) {
      return "Shift 4";
    } else if (getMatchTime() > 0) {
      return "Endgame";
    } else {
      return "Post-Match";
    }
  }
}
