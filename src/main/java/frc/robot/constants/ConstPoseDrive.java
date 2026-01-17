// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.DriverStateMachine.DriverState;

/**
 * Provides configuration constants for pose-based driving logic.
 * <p>
 * The {@code ConstPoseDrive} class contains static configuration groups used to
 * control
 * robot movement towards specific poses on the field. Each
 * {@link PoseDriveGroup} instance
 * encapsulates parameters such as minimum distance before driving, target pose
 * groups,
 * driving and snapping states, axis locks, and tolerances for distance and
 * rotation.
 * <p>
 * Example usage:
 * 
 * <pre>
 * ConstPoseDrive.PoseDriveGroup group = ConstPoseDrive.EXAMPLE_POSE_DRIVE_GROUP;
 * // Use group.minDistanceBeforeDrive, group.targetPoseGroup, etc. in drive
 * // logic
 * </pre>
 * <p>
 * The {@code EXAMPLE_POSE_DRIVE_GROUP} provides a sample configuration for
 * reference.
 */
public class ConstPoseDrive {
  /**
   * Configuration group for pose-based driving.
   * <p>
   * Each {@code PoseDriveGroup} defines a set of target poses and driving
   * parameters,
   * including minimum distance before initiating drive, driving and snapping
   * states,
   * axis locks, and tolerances for distance and rotation.
   */
  public static class PoseDriveGroup {
    public Distance minDistanceBeforeDrive;
    public List<Pose2d> targetPoseGroup;
    public DriverState driveState;
    public DriverState snapState;
    public boolean lockX;
    public boolean lockY;
    public Distance distanceTolerance = Units.Inches.of(0);
    public Angle rotationTolerance = Units.Degrees.of(0);
  }

  public static final PoseDriveGroup EXAMPLE_POSE_DRIVE_GROUP = new PoseDriveGroup();

  static {
    EXAMPLE_POSE_DRIVE_GROUP.minDistanceBeforeDrive = Units.Inches.of(100);
    EXAMPLE_POSE_DRIVE_GROUP.targetPoseGroup = ConstField.FieldElementGroups.RESET_POSE_SET.getAll();
    EXAMPLE_POSE_DRIVE_GROUP.driveState = DriverState.EXAMPLE_POSE_DRIVE;
    EXAMPLE_POSE_DRIVE_GROUP.snapState = DriverState.EXAMPLE_ROTATION_SNAP;
    EXAMPLE_POSE_DRIVE_GROUP.lockX = false;
    EXAMPLE_POSE_DRIVE_GROUP.lockY = false;
    EXAMPLE_POSE_DRIVE_GROUP.distanceTolerance = Units.Inches.of(1);
    EXAMPLE_POSE_DRIVE_GROUP.rotationTolerance = Units.Degrees.of(1);
  }
}
