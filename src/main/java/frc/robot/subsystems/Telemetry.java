// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import java.util.Currency;
import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.RobotController;

@Logged
public class Telemetry extends SubsystemBase {
  /** Creates a new Telemetry. */
  double timeBefore = 6;
  double shift1Time = 105;
  double autoTime = 140;
  double transitionShiftTime = 130;
  double shift2Time = 80;
  double shift3Time = 55;
  double shift4Time = 30;
  double endgameTime = 0;

  private String gameData = "";

  public Telemetry() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (gameData.isEmpty()) {
      gameData = DriverStation.getGameSpecificMessage();
    }

  }

  public boolean redIsNotFirst() {
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
    return redInactiveFirst;
  }

  public double getMatchTime() {
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

  public boolean hubsIsSwitching() {
    double matchTime = getMatchTime();
    if ((matchTime < transitionShiftTime + timeBefore && matchTime > transitionShiftTime)
        || (matchTime < shift1Time + timeBefore && matchTime > shift1Time)
        || (matchTime < shift2Time + timeBefore && matchTime > shift2Time)
        || (matchTime < shift3Time + timeBefore && matchTime > shift3Time)
        || (matchTime < shift4Time + timeBefore && matchTime > shift4Time)) {
      return true;
    }
    return false;
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

  public boolean ourShiftFirst() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      return false;
    }
    if (gameData.isEmpty()) {
      return false;
    }

    if (((alliance.get().equals(Alliance.Red) && redIsNotFirst() == false)
        ||
        (alliance.get().equals(Alliance.Blue) && redIsNotFirst() == true))
        && getMatchTime() < autoTime
        && getMatchTime() > transitionShiftTime) {
      return true;
    }
    return false;
  }

  public Voltage batteryVoltage() {
    return RobotController.getMeasureBatteryVoltage();
  }

  public boolean isBrownedOut() {
    return RobotController.isBrownedOut();
  }

}