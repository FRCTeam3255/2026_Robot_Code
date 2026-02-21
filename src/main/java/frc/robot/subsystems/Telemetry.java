// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telemetry extends SubsystemBase {
  /** Creates a new Telemetry. */
  public Telemetry() {
  }

  boolean autoTime = getMatchTime() > 140;
  boolean transitionShiftTime = getMatchTime() > 130;
  boolean shift1Time = getMatchTime() > 105;
  boolean shift2Time = getMatchTime() > 80;
  boolean shift3Time = getMatchTime() > 55;
  boolean shift4Time = getMatchTime() > 30;
  boolean endgameTime = getMatchTime() > 0;
  boolean shiftChangeTime = getMatchTime() < 134 && getMatchTime() > 130
      || getMatchTime() < 109 && getMatchTime() > 105
      || getMatchTime() < 84 && getMatchTime() > 80
      || getMatchTime() < 59 && getMatchTime() > 55
      || getMatchTime() < 34 && getMatchTime() > 30;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    } else if (shift1Time) {
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
    if (autoTime) {
      return MatchPeriods.AUTO.toString();
    } else if (transitionShiftTime) {
      return MatchPeriods.TRANSITION_SHIFT.toString();
    } else if (shift1Time) {
      return MatchPeriods.SHIFT_1.toString();
    } else if (shift2Time) {
      return MatchPeriods.SHIFT_2.toString();
    } else if (shift3Time) {
      return MatchPeriods.SHIFT_3.toString();
    } else if (shift4Time) {
      return MatchPeriods.SHIFT_4.toString();
    } else if (endgameTime) {
      return MatchPeriods.ENDGAME.toString();
    } else {
      return MatchPeriods.POST_MATCH.toString();
    }
  }

  public boolean hubsIsSwitching() {
    if (shiftChangeTime) {
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

    if (autoTime) {
      // Transition shift, hub is active.
      return true;
    } else if (shift1Time) {
      // Shift 1
      return shift1Active;
    } else if (shift2Time) {
      // Shift 2
      return !shift1Active;
    } else if (shift3Time) {
      // Shift 3
      return shift1Active;
    } else if (shift4Time) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }
}
