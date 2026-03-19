package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
  public static final LinearVelocity ground_intaking = Units.MetersPerSecond.of(0.61);

  public static final class Poses {
    public static final Pose2d D_side_climb_starting_pose = new Pose2d(2.891, 4.205, Rotation2d.fromRadians(-1.571));
    public static final Pose2d D_side_tower = new Pose2d(1.484, 4.205, Rotation2d.fromRadians(-1.571));
    public static final Pose2d Hub = new Pose2d(4.629, 4.032, Rotation2d.kZero);
    public static final Pose2d Hub_front = new Pose2d(2.3, 4, Rotation2d.kZero);
    public static final Pose2d O_side_climb_starting_pose = new Pose2d(2.854, 3.427, Rotation2d.fromRadians(-1.571));
    public static final Pose2d O_side_tower = new Pose2d(1.484, 3.427, Rotation2d.fromRadians(-1.571));
    public static final Pose2d OppBump = new Pose2d(3.6, 2.6, Rotation2d.fromRadians(1.571));
    public static final Pose2d OppHub = new Pose2d(2.7, 3, Rotation2d.fromRadians(0.524));
    public static final Pose2d OppNeutral = new Pose2d(7.7, 2.8, Rotation2d.fromRadians(4.538));
    public static final Pose2d bump = new Pose2d(3.6, 5.7, Rotation2d.fromRadians(4.712));
    public static final Pose2d bump_forward = new Pose2d(3.592, 6.054, Rotation2d.kZero);
    public static final Pose2d depot = new Pose2d(0.677, 5.994, Rotation2d.kZero);
    public static final Pose2d depot_front = new Pose2d(2.458, 5.825, Rotation2d.kZero);
    public static final Pose2d depot_side_neutral = new Pose2d(7.601, 5.857, Rotation2d.fromRadians(1.588));
    public static final Pose2d depot_side_neutral_shoot = new Pose2d(2.779, 5.269, Rotation2d.kZero);
    public static final Pose2d depot_side_trench = new Pose2d(3.683, 7.568, Rotation2d.fromRadians(1.571));
    public static final Pose2d first_oside_neutral_intake_end = new Pose2d(7.227, 2.778, Rotation2d.fromRadians(1.571));
    public static final Pose2d first_oside_neutral_intake_end = new Pose2d(7.227, 2.778, Rotation2d.fromRadians(1.571));
    public static final Pose2d hub_left = new Pose2d(2.6, 5.1, Rotation2d.fromRadians(5.76));
    public static final Pose2d neutral = new Pose2d(7.7, 5.2, Rotation2d.fromRadians(1.71));
    public static final Pose2d outpost = new Pose2d(0.733, 0.445, Rotation2d.kZero);
    public static final Pose2d outpost_front = new Pose2d(1.779, 2.156, Rotation2d.fromRadians(0.569));
    public static final Pose2d outpost_intake = new Pose2d(0.696, 0.434, Rotation2d.kZero);
    public static final Pose2d outpost_side_neutral = new Pose2d(7.604, 2.375, Rotation2d.fromRadians(-1.571));
    public static final Pose2d outpost_side_neutral_shoot = new Pose2d(2.779, 2.582, Rotation2d.kZero);
    public static final Pose2d outpost_side_neutral_zone = new Pose2d(6.045, 2.249, Rotation2d.fromRadians(-1.048));
    public static final Pose2d outpost_side_trench = new Pose2d(3.612, 0.412, Rotation2d.kZero);
    public static final Pose2d outpost_side_trench_nuetral = new Pose2d(3.684, 0.499, Rotation2d.fromRadians(4.712));
    public static final Pose2d preload_shoot = new Pose2d(2.512, 4.004, Rotation2d.kZero);
    public static final Pose2d second_oside_neutral_intake_end = new Pose2d(6.594, 2.329, Rotation2d.kZero);

    private Poses() {
    }
  }

  private ChoreoVars() {
  }
}