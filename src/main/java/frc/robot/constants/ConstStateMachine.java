package frc.robot.constants;

import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.states.EjectingHopper;
import frc.robot.commands.states.Intaking;
import frc.robot.commands.states.None;
import frc.robot.commands.states.RetractIntake;
import frc.robot.commands.states.ReverseShooter;
import frc.robot.commands.states.Shooting;
import frc.robot.commands.states.Climbing.ClimbingL1;
import frc.robot.commands.states.Climbing.ClimbingL2_3;
import frc.robot.commands.states.Climbing.PrepClimb;
import frc.robot.commands.states.Climbing.Unclimb;
import frc.robot.commands.states.PrepShoots.PrepAnywhere;
import frc.robot.commands.states.PrepShoots.PrepCorner;
import frc.robot.commands.states.PrepShoots.PrepDepot;
import frc.robot.commands.states.PrepShoots.PrepHub;
import frc.robot.commands.states.PrepShoots.PrepNeutralToAlliance;
import frc.robot.commands.states.PrepShoots.PrepNonOutpost;
import frc.robot.commands.states.PrepShoots.PrepOpponentToAlliance;
import frc.robot.commands.states.PrepShoots.PrepTower;
import frc.robot.commands.states.PrepShoots.PrepTrench;

public final class ConstStateMachine {
  private ConstStateMachine() {
  }

  public enum RobotState {
    NONE(None::new),
    INTAKING(Intaking::new),
    PREP_CORNER(PrepCorner::new),
    PREP_DEPOT(PrepDepot::new),
    PREP_HUB(PrepHub::new),
    PREP_TRENCH(PrepTrench::new),
    PREP_ANYWHERE(PrepAnywhere::new),
    PREP_NON_OUTPOST(PrepNonOutpost::new),
    PREP_TOWER(PrepTower::new),
    PREP_NEUTRAL_TO_ALLIANCE(PrepNeutralToAlliance::new),
    PREP_OPPONENT_TO_ALLIANCE(PrepOpponentToAlliance::new),
    SHOOTING(Shooting::new),
    EJECTING_HOPPER(EjectingHopper::new),
    REVERSING_SHOOTER(ReverseShooter::new),
    PREP_CLIMB_L1(PrepClimb::new),
    CLIMBING_L1(ClimbingL1::new),
    CLIMBING_L2_3(ClimbingL2_3::new),
    UNCLIMB_L1(Unclimb::new),
    RETRACT_INTAKE(RetractIntake::new);

    public final Supplier<Command> commandFactory;

    RobotState(Supplier<Command> commandFactory) {
      this.commandFactory = commandFactory;
    }

    /** Requests this state on the robot's state machine, validating the transition. */
    public Command TRY() {
      return RobotContainer.stateMachineInstance.tryState(this);
    }
  }

  public static final Map<RobotState, Set<RobotState>> LEGAL_MOVES = Map.ofEntries(
      Map.entry(RobotState.NONE, Set.of(
          RobotState.NONE,
          RobotState.UNCLIMB_L1,
          RobotState.INTAKING,
          RobotState.EJECTING_HOPPER,
          RobotState.REVERSING_SHOOTER,
          RobotState.SHOOTING)),
      Map.entry(RobotState.INTAKING, Set.of(
          RobotState.NONE,
          RobotState.RETRACT_INTAKE,
          RobotState.PREP_CLIMB_L1,
          RobotState.UNCLIMB_L1,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.PREP_ANYWHERE, Set.of(
          RobotState.NONE,
          RobotState.INTAKING,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.PREP_DEPOT, Set.of(
          RobotState.NONE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.PREP_HUB, Set.of(
          RobotState.NONE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.PREP_TRENCH, Set.of(
          RobotState.NONE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.PREP_CORNER, Set.of(
          RobotState.NONE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.PREP_NON_OUTPOST, Set.of(
          RobotState.NONE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.PREP_TOWER, Set.of(
          RobotState.NONE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.PREP_NEUTRAL_TO_ALLIANCE, Set.of(
          RobotState.NONE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.PREP_OPPONENT_TO_ALLIANCE, Set.of(
          RobotState.NONE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.SHOOTING, Set.of(
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_TOWER,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.EJECTING_HOPPER, Set.of(
          RobotState.NONE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.REVERSING_SHOOTER, Set.of(
          RobotState.NONE,
          RobotState.PREP_DEPOT,
          RobotState.PREP_HUB,
          RobotState.PREP_ANYWHERE,
          RobotState.PREP_TRENCH,
          RobotState.PREP_CORNER,
          RobotState.PREP_TOWER,
          RobotState.PREP_NON_OUTPOST,
          RobotState.PREP_NEUTRAL_TO_ALLIANCE,
          RobotState.PREP_OPPONENT_TO_ALLIANCE)),
      Map.entry(RobotState.PREP_CLIMB_L1, Set.of(RobotState.NONE)),
      Map.entry(RobotState.CLIMBING_L1, Set.of(RobotState.PREP_CLIMB_L1, RobotState.UNCLIMB_L1)),
      Map.entry(RobotState.UNCLIMB_L1, Set.of(RobotState.CLIMBING_L1)),
      Map.entry(RobotState.CLIMBING_L2_3, Set.of(RobotState.CLIMBING_L1, RobotState.CLIMBING_L2_3)),
      Map.entry(RobotState.RETRACT_INTAKE, Set.of(RobotState.NONE)));
}
