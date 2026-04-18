// spotless:off
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
    public static final Angle dside_over_bumb_angle = Units.Radians.of(-0.7853982);
    public static final LinearVelocity ground_intaking = Units.MetersPerSecond.of(1.6764);
    public static final Angle oside_over_bump_angle = Units.Radians.of(0.7853982);
    public static final LinearVelocity over_bump = Units.MetersPerSecond.of(2.1336);

    public static final class Poses {
        public static final Pose2d D_side_climb_starting_pose = new Pose2d(2.2286003, 4.2053921, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d D_side_tower = new Pose2d(1.4841553, 4.2053921, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d Hub = new Pose2d(4.6292157, 4.0317845, Rotation2d.fromRadians(0));
        public static final Pose2d Hub_front = new Pose2d(2.3, 4, Rotation2d.fromRadians(0));
        public static final Pose2d O_side_climb_starting_pose = new Pose2d(2.1969085, 3.427, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d O_side_tower = new Pose2d(1.4841553, 3.427, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d OppBump = new Pose2d(3.6, 2.6, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d OppHub = new Pose2d(2.7, 3, Rotation2d.fromRadians(0.5235988));
        public static final Pose2d OppNeutral = new Pose2d(7.7, 2.8, Rotation2d.fromRadians(4.5378561));
        public static final Pose2d against_hub = new Pose2d(3.5972879, 4.0099173, Rotation2d.fromRadians(0));
        public static final Pose2d bump = new Pose2d(3.6, 5.7, Rotation2d.fromRadians(4.712389));
        public static final Pose2d bump_forward = new Pose2d(3.5924933, 6.0540094, Rotation2d.fromRadians(0));
        public static final Pose2d depot = new Pose2d(0.6772127, 5.9943109, Rotation2d.fromRadians(0));
        public static final Pose2d depot_front = new Pose2d(2.4582205, 5.8251648, Rotation2d.fromRadians(0));
        public static final Pose2d depot_side_midline_neutral = new Pose2d(7.8755808, 5.9022393, Rotation2d.fromRadians(1.5877437));
        public static final Pose2d depot_side_neutral = new Pose2d(7.4477515, 5.7073744, Rotation2d.fromRadians(1.5877437));
        public static final Pose2d depot_side_neutral_shoot = new Pose2d(2.7789404, 5.4687225, Rotation2d.fromRadians(-0.7853982));
        public static final Pose2d depot_side_offset_neutral = new Pose2d(8.385685, 5.9022393, Rotation2d.fromRadians(1.5877437));
        public static final Pose2d depot_side_trench = new Pose2d(3.6833227, 7.5683317, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d first_dside_neutral_intake_end = new Pose2d(6.920249, 5.3060495, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d first_oside_neutral_intake_end = new Pose2d(7.017382, 2.3589698, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d hub_left = new Pose2d(2.6, 5.1, Rotation2d.fromRadians(5.7595865));
        public static final Pose2d neutral = new Pose2d(7.7, 5.2, Rotation2d.fromRadians(1.7104227));
        public static final Pose2d outpost = new Pose2d(0.7329863, 0.4449631, Rotation2d.fromRadians(0));
        public static final Pose2d outpost_front = new Pose2d(1.1765397, 2.0080771, Rotation2d.fromRadians(0.6022872));
        public static final Pose2d outpost_intake = new Pose2d(0.6961642, 0.4335097, Rotation2d.fromRadians(0));
        public static final Pose2d outpost_side_midline_neutral = new Pose2d(7.8807001, 2.1492248, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d outpost_side_neutral = new Pose2d(7.4477515, 2.5050711, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d outpost_side_neutral_shoot = new Pose2d(2.7789404, 2.5817356, Rotation2d.fromRadians(0.7853982));
        public static final Pose2d outpost_side_neutral_zone = new Pose2d(6.0448093, 2.2489808, Rotation2d.fromRadians(-1.0480071));
        public static final Pose2d outpost_side_offset_neutral = new Pose2d(8.385685, 2.1492248, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d outpost_side_trench = new Pose2d(3.6123929, 0.4124934, Rotation2d.fromRadians(0));
        public static final Pose2d outpost_side_trench_nuetral = new Pose2d(3.6836977, 0.4991045, Rotation2d.fromRadians(4.712389));
        public static final Pose2d preload_shoot = new Pose2d(2.5119886, 4.0041704, Rotation2d.fromRadians(0));
        public static final Pose2d second_dside_neutral_intake_end = new Pose2d(6.619998, 5.5248582, Rotation2d.fromRadians(0));
        public static final Pose2d second_oside_neutral_intake_end = new Pose2d(6.5943604, 2.3292248, Rotation2d.fromRadians(0));
    }
}
// spotless:on
