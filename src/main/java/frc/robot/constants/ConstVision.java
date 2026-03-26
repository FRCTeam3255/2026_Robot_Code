// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/**
 * Contains constants related to vision processing for the robot.
 * <p>
 * This class provides standard deviation values used by the pose estimator
 * for vision data, including position (in meters) and heading (in radians).
 */
public class ConstVision {

  public static class IMUMode {
    /**
     * No internal IMU processing. MT2 uses interpolated yaw from robot's gyro sent
     * via SetRobotOrientation().
     */
    public static final int EXTERNAL_ONLY = 0;
    /**
     * Internal IMU offset is calibrated to match external yaw each frame (seeding).
     * MT2 still uses external yaw for botpose.
     */
    public static final int EXTERNAL_SEED = 1;
    /** Uses internal IMU's fused yaw only. No external input required. */
    public static final int INTERNAL_ONLY = 2;
    /**
     * Complementary filter fuses internal IMU with MT1 vision yaw. When MT1 gets a
     * valid pose, it slowly corrects internal IMU drift.
     */
    public static final int INTERNAL_MT1_ASSIST = 3;
    /**
     * Complementary filter fuses internal IMU with external yaw from
     * SetRobotOrientation(). This is the recommended mode, as the internal IMU's
     * 1khz update rate is utilized for frame-by-frame motion while the robot's IMU
     * corrects for any drift over time.
     */
    public static final int INTERNAL_EXTERNAL_ASSIST = 4;
  }

  public static final String LIMELIGHT_RIGHT_NAME = "limelight-right";
  public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
  public static final String LIMELIGHT_TOP_NAME = "limelight-top";

  /**
   * <p>
   * Pose estimator standard deviation for vision data
   * <p>
   * <b>Units:</b> Meters
   */

  public static final double STD_DEVS_POS = 0.7;
  /**
   * <p>
   * Pose estimator standard deviation for vision data
   * </p>
   * <b>Units:</b> Radians
   */

  public static final double STD_DEVS_HEADING = 9999999;

  /**
   * <p>
   * Pose estimator standard deviation for vision data
   * </p>
   * <b>Units:</b> Meters
   */
  public static final double MEGA_TAG1_STD_DEVS_POSITION = .3;

  public static final double MEGA_TAG1_STD_DEVS_HEADING = .1;
  /**
   * <p>
   * Maximum rate of rotation before we begin rejecting pose updates
   * </p>
   */
  public static final AngularVelocity MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond.of(720);

  /**
   * The area that one tag (if its the only tag in the update) needs to exceed
   * before being accepted
   */
  public static final double AREA_THRESHOLD_FRONT = 0.1;
  public static final double AREA_THRESHOLD_BACK = 0.05;
  public static final int DisabledThrottle = 100;
  public static final int TeleopThrottle = 0;
  public static final Boolean USE_MEGA_TAG_2 = false;

  // The below values are accounted for in the limelight interface, NOT in code
  public static class LimelightRight {
    public static final Distance LL_FORWARD = Units.Meters.of(0.269494);
    public static final Distance LL_RIGHT = Units.Meters.of(0.307594);
    public static final Distance LL_UP = Units.Meters.of(0.211328);

    public static final Angle LL_ROLL = Units.Degrees.of(180);
    public static final Angle LL_PITCH = Units.Degrees.of(23.17);
    public static final Angle LL_YAW = Units.Degrees.of(51.25);
  }

  public static class LimelightLeft {
    public static final Distance LL_FORWARD = Units.Meters.of(0.269494);
    public static final Distance LL_RIGHT = Units.Meters.of(-0.307594);
    public static final Distance LL_UP = Units.Meters.of(0.211328);

    public static final Angle LL_ROLL = Units.Degrees.of(180);
    public static final Angle LL_PITCH = Units.Degrees.of(23.17);
    public static final Angle LL_YAW = Units.Degrees.of(-51.25);

  }

  public static class LimelightBack {
    public static final Distance LL_FORWARD = Units.Meters.of(0.3429);
    public static final Distance LL_RIGHT = Units.Meters.of(0);
    public static final Distance LL_UP = Units.Meters.of(0.2921);

    public static final Angle LL_ROLL = Units.Degrees.of(0);
    public static final Angle LL_PITCH = Units.Degrees.of(-20);
    public static final Angle LL_YAW = Units.Degrees.of(0);
  }

}
