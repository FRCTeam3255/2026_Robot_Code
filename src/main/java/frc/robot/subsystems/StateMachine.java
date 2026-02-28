// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.states.Climbing.*;
import frc.robot.commands.states.*;
import frc.robot.commands.states.Climbing.PrepClimb;
import frc.robot.commands.states.PrepShoots.*;

@Logged
public class StateMachine extends SubsystemBase {
  public static RobotState currentRobotState;

  /** Creates a new StateMachine. */
  public StateMachine() {
    currentRobotState = RobotState.NONE;
  }

  public void setRobotState(RobotState robotState) {
    currentRobotState = robotState;
  }

  public RobotState getRobotState() {
    return currentRobotState;
  }

  public Command tryState(RobotState desiredState) {
    switch (desiredState) {
      case NONE:
        switch (currentRobotState) {
          case NONE:
          case UNCLIMB_L1:
          case INTAKING:
          case EJECTING_HOPPER:
          case REVERSING_SHOOTER:
          case SHOOTING:
            return new None();
        }
        break;

      case INTAKING:
        switch (currentRobotState) {
          case NONE:
          case DEFENSE:
          case PREP_CLIMB_L1:
            return new Intaking();
        }
        break;

      case PREP_ANYWHERE:
        switch (currentRobotState) {
          case NONE:
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_TOWER:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new PrepAnywhere();
        }
        break;

      case PREP_DEPOT:
        switch (currentRobotState) {
          case NONE:
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_TOWER:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new PrepDepot();
        }
        break;

      case PREP_TRENCH:
        switch (currentRobotState) {
          case NONE:
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_TOWER:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new PrepTrench();
        }
        break;

      case PREP_OUTPOST:
        switch (currentRobotState) {
          case NONE:
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_TOWER:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new PrepOutpost();
        }
        break;

      case PREP_NON_OUTPOST:
        switch (currentRobotState) {
          case NONE:
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_TOWER:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new PrepNonOutpost();
        }
        break;

      case PREP_TOWER:
        switch (currentRobotState) {
          case NONE:
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new PrepTower();
        }
        break;

      case PREP_NEUTRAL_TO_ALLIANCE:
        switch (currentRobotState) {
          case NONE:
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_TOWER:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new PrepNeutralToAlliance();
        }
        break;

      case PREP_OPPONENT_TO_ALLIANCE:
        switch (currentRobotState) {
          case NONE:
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_TOWER:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new PrepOpponentToAlliance();
        }
        break;

      case SHOOTING:
        switch (currentRobotState) {
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new Shooting();
        }
        break;

      case EJECTING_HOPPER:
        switch (currentRobotState) {
          case NONE:
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new EjectingHopper();
        }
        break;

      case REVERSING_SHOOTER:
        switch (currentRobotState) {
          case NONE:
          case PREP_DEPOT:
          case PREP_ANYWHERE:
          case PREP_TRENCH:
          case PREP_OUTPOST:
          case PREP_NON_OUTPOST:
          case PREP_NEUTRAL_TO_ALLIANCE:
          case PREP_OPPONENT_TO_ALLIANCE:
            return new ReverseShooter();
        }
        break;

      case PREP_CLIMB_L1:
        switch (currentRobotState) {
          case NONE:
            return new PrepClimb();
        }
        break;

      case CLIMBING_L1:
        switch (currentRobotState) {
          case PREP_CLIMB_L1:
            return new ClimbingL1();
        }
        break;

      case UNCLIMB_L1:
        switch (currentRobotState) {
          case CLIMBING_L1:
            return new Unclimb();
        }
        break;

      case CLIMBING_L2_3:
        switch (currentRobotState) {
          case CLIMBING_L1:
          case CLIMBING_L2_3:
            return new ClimbingL2_3();
        }
        break;

      case DEFENSE:
        switch (currentRobotState) {
          case NONE:
            return new Defense();
        }

    }

    return Commands
        .print("ITS SO OVER D: Invalid State Provided, Blame Eli. Attempted to go to: " + desiredState.toString()
            + " while at " + currentRobotState.toString());
  }

  public enum RobotState {
    NONE,
    INTAKING,
    PREP_OUTPOST,
    PREP_DEPOT,
    PREP_TRENCH,
    PREP_ANYWHERE,
    PREP_NON_OUTPOST,
    PREP_TOWER,
    PREP_NEUTRAL_TO_ALLIANCE,
    PREP_OPPONENT_TO_ALLIANCE,
    SHOOTING,
    EJECTING_HOPPER,
    REVERSING_SHOOTER,
    PREP_CLIMB_L1,
    CLIMBING_L1,
    CLIMBING_L2_3,
    UNCLIMB_L1,
    DEFENSE,
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
