// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.ConstStateMachine.LEGAL_MOVES;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConstStateMachine.RobotState;
import java.util.Set;

@Logged
public class StateMachine extends SubsystemBase {
  private RobotState currentRobotState = RobotState.NONE;

  public void setRobotState(RobotState robotState) {
    currentRobotState = robotState;
  }

  public RobotState getRobotState() {
    return currentRobotState;
  }

  public boolean canGoTo(RobotState desiredState) {
    return LEGAL_MOVES.get(desiredState).contains(currentRobotState);
  }

  public Command tryState(RobotState desiredState) {
    return Commands.defer(
        () -> {
          if (!canGoTo(desiredState)) {
            return Commands.print(
                "ITS SO OVER D: Invalid State Provided, Blame Eli. Attempted to go to: "
                    + desiredState
                    + " while at "
                    + currentRobotState);
          }
          return desiredState.commandFactory.get()
              .beforeStarting(() -> currentRobotState = desiredState);
        },
        Set.of(this));
  }
}
