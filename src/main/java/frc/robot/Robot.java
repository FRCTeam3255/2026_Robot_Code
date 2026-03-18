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
  private final StringTopic selectedTabTopic = NetworkTableInstance.getDefault()
      .getStringTopic("/Elastic/SelectedTab");
  private final StringPublisher selectedTabPublisher = selectedTabTopic
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
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_TOP_NAME, ConstVision.DisabledThrottle);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.IMUMode.EXTERNAL_SEED);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.IMUMode.EXTERNAL_SEED);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_TOP_NAME, ConstVision.IMUMode.EXTERNAL_SEED);
  }

  @Override
  public void disabledPeriodic() {
    ConstField.ALLIANCE = DriverStation.getAlliance();
    SmartDashboard.putString("ALLIANCE", ConstField.ALLIANCE.toString());
    double yaw = m_robotContainer.drivetrainInstance.getPose().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation(ConstVision.LIMELIGHT_RIGHT_NAME,
        yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(ConstVision.LIMELIGHT_LEFT_NAME,
        yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(ConstVision.LIMELIGHT_TOP_NAME,
        yaw, 0, 0, 0, 0, 0);

  }

  @Override
  public void disabledExit() {
    m_robotContainer.manualZeroSubsystems.cancel();
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.TeleopThrottle);
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.TeleopThrottle);
    LimelightHelpers.SetThrottle(ConstVision.LIMELIGHT_TOP_NAME, ConstVision.TeleopThrottle);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.IMUMode.INTERNAL_ONLY);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.IMUMode.INTERNAL_ONLY);
    LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_TOP_NAME, ConstVision.IMUMode.INTERNAL_ONLY);
    m_robotContainer.addVisionMeasurement().schedule();
  }

  @Override
  public void autonomousInit() {
    selectTab("Autonomous");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    bothSubsystemsZeroed = m_robotContainer.allZeroed();

    if (bothSubsystemsZeroed && m_autonomousCommand != null) {
      Commands.deferredProxy(() -> m_autonomousCommand).schedule();
    } else if (m_autonomousCommand != null) {
      m_robotContainer.makeZeroSubsystemsCommand().andThen(Commands.deferredProxy(() -> m_autonomousCommand))
          .schedule();
      m_autonomousCommand.schedule();
    } else {
      m_robotContainer.makeZeroSubsystemsCommand().schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  public void selectTab(String tabName) {
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
      m_robotContainer.makeZeroSubsystemsCommand().schedule();
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
}
