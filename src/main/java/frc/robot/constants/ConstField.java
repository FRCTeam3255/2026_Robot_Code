// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Provides constants and utilities for field dimensions and alliance-specific pose management.
 * <p>
 * This class defines the field size and contains logic for handling poses that need to be mirrored
 * between the blue and red alliances. The {@link Pose2dAllianceSet} inner class allows you to define
 * a set of poses for the blue alliance and automatically generates the corresponding mirrored poses
 * for the red alliance, as well as a combined set for both alliances.
 * <p>
 * The field mirroring functionality is essential for autonomous routines and path planning, ensuring
 * that robot positions and trajectories can be easily adapted for both sides of the field. The
 * {@link #isRedAlliance()} method can be used to determine the current alliance and select the
 * appropriate pose set.
 */
public class ConstField {
  public static Optional<Alliance> ALLIANCE = Optional.empty();

  // Wrapper for blue-side Pose2d arrays with helpers for red/all
  /**
   * Represents a set of {@link Pose2d} objects for alliance-specific field positions.
   * <p>
   * This class stores a list of blue-side poses and automatically computes the corresponding
   * red-side poses by mirroring them across the field. It provides access to the blue, red,
   * and combined lists of poses, allowing code to easily retrieve poses for the current alliance.
   * <p>
   * The transformation to red-side poses is performed by reflecting the blue-side pose
   * across the field's length and width, and rotating the heading by 180 degrees.
   */
  public static final class Pose2dAllianceSet {
    private final List<Pose2d> blue;
    private final List<Pose2d> red;
    private final List<Pose2d> all;

    Pose2dAllianceSet(Pose2d... bluePoses) {
      this.blue = List.of(bluePoses);
      this.red = this.blue.stream().map(Pose2dAllianceSet::toRed).toList();

      var combined = new java.util.ArrayList<Pose2d>(this.blue.size() * 2);
      combined.addAll(this.blue);
      combined.addAll(this.red);
      this.all = List.copyOf(combined);
    }

    public List<Pose2d> getBlue() {
      return blue;
    }

    public List<Pose2d> getRed() {
      return red;
    }

    public List<Pose2d> getAll() {
      return all;
    }

    private static Pose2d toRed(Pose2d bluePose) {
      return new Pose2d(
          FIELD_LENGTH.in(Units.Meters) - bluePose.getX(),
          FIELD_WIDTH.in(Units.Meters) - bluePose.getY(),
          bluePose.getRotation().plus(Rotation2d.k180deg));
    }

    static Pose2dAllianceSet of(Pose2d... bluePoses) {
      return new Pose2dAllianceSet(bluePoses);
    }

    static Pose2dAllianceSet concat(Pose2dAllianceSet... sets) {
      var combined = new java.util.ArrayList<Pose2d>();
      for (Pose2dAllianceSet set : sets) {
        combined.addAll(set.getBlue());
      }
      return new Pose2dAllianceSet(combined.toArray(new Pose2d[0]));
    }
  }

  public static final Distance FIELD_LENGTH = Units.Feet.of(57).plus(Units.Inches.of(6.875));
  public static final Distance FIELD_WIDTH = Units.Feet.of(26).plus(Units.Inches.of(5));

  /**
   * Boolean that controls when the path will be mirrored for the red
   * alliance. This will flip the path being followed to the red side of the
   * field.
   * The origin will remain on the Blue side.
   * 
   * @return If we are currently on Red alliance. Will return false if no alliance
   *         is found
   */
  public static boolean isRedAlliance() {
    var alliance = ALLIANCE;
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  };

  public static class FieldElements {
    private static final Pose2d RESET_POSE = new Pose2d(0, 0, new Rotation2d());

  }

  public static class FieldElementGroups {
    public static final Pose2dAllianceSet RESET_POSE_SET = new Pose2dAllianceSet(
        FieldElements.RESET_POSE);
  }

  public static final Pose2d WORKSHOP_STARTING_POSE = new Pose2d(5.98, 2.60, new Rotation2d(0));
}
