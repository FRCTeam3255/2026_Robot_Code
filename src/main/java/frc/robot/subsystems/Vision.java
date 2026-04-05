// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConstVision;

@Logged
public class Vision extends SubsystemBase {
  PoseEstimate lastEstimateRight = new PoseEstimate();
  PoseEstimate lastEstimateLeft = new PoseEstimate();
  PoseEstimate lastEstimateBack = new PoseEstimate();
  private boolean visionEnabled = true;

  // Not logged, as they turn to false immediately after being read
  @NotLogged
  boolean newRightEstimate = false;
  @NotLogged
  boolean newLeftEstimate = false;
  @NotLogged
  boolean newBackEstimate = false;

  Pose2d rightPose = new Pose2d();
  Pose2d leftPose = new Pose2d();
  Pose2d backPose = new Pose2d();
  int rightTagCount = 0;
  int leftTagCount = 0;
  int backTagCount = 0;

  String limelightInUse = LL_INUSE.NONE.toString();

  public enum LL_INUSE {
    RIGHT, LEFT, BACK, NONE
  }

  private boolean useMegaTag2 = ConstVision.USE_MEGA_TAG_2;

  public Vision() {
  }

  public PoseEstimate[] getLastPoseEstimates() {
    return new PoseEstimate[] { lastEstimateRight, lastEstimateLeft, lastEstimateBack };
  }

  public void setMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
  }

  public void setIMUAssistMode(boolean useAssist) {
    if (useAssist == true) {
      LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.IMUMode.INTERNAL_MT1_ASSIST);
      LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.IMUMode.INTERNAL_MT1_ASSIST);
      LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_TOP_NAME, ConstVision.IMUMode.INTERNAL_MT1_ASSIST);
    } else {
      LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_RIGHT_NAME, ConstVision.IMUMode.EXTERNAL_SEED);
      LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_LEFT_NAME, ConstVision.IMUMode.EXTERNAL_SEED);
      LimelightHelpers.SetIMUMode(ConstVision.LIMELIGHT_TOP_NAME, ConstVision.IMUMode.EXTERNAL_SEED);
    }
  }

  /**
   * Determines if a given pose estimate should be rejected.
   * 
   * 
   * @param poseEstimate The pose estimate to check
   * @param gyroRate     The current rate of rotation observed by our gyro.
   * 
   * @return True if the estimate should be rejected
   */

  public boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate, double areaThreshold) {
    // Angular velocity is too high to have accurate vision
    if (gyroRate.compareTo(ConstVision.MAX_ANGULAR_VELOCITY) > 0) {
      return true;
    }

    if (poseEstimate == null
        || poseEstimate.pose == null
        || poseEstimate.pose.getTranslation() == null
        || !Double.isFinite(poseEstimate.pose.getTranslation().getX())
        || !Double.isFinite(poseEstimate.pose.getTranslation().getY())) {
      System.err.println("********REJECTING POSE ESTIMATE: pose contained non-finite X or Y (NaN/Inf)********");
      return true;
    }
    // No tags :<
    if (poseEstimate.tagCount == 0) {
      return true;
    }

    // If MegaTag 2 do not reject if other items passed
    if (useMegaTag2) {
      return false;
    }

    // If MegaTag 1 have additional checks
    // 1 Tag with a large area
    if (poseEstimate.tagCount == 1 && poseEstimate.avgTagArea > areaThreshold) {
      return false;
      // 2 tags or more
    } else if (poseEstimate.tagCount > 1) {
      return false;
    }

    return true;
  }

  /**
   * Updates the current pose estimates for the left and right of the robot using
   * data from Limelight cameras.
   *
   * @param gyroRate The current angular velocity of the robot, used to validate
   *                 the pose estimates.
   *
   *                 This method retrieves pose estimates from two Limelight
   *                 cameras (left and right) and updates the
   *                 corresponding pose estimates if they are valid. The method
   *                 supports two modes of operation:
   *                 one using MegaTag2 and one without. The appropriate pose
   *                 estimate retrieval method is chosen
   *                 based on the value of the `useMegaTag2` flag.
   *
   *                 If the retrieved pose estimates are valid and not rejected
   *                 based on the current angular velocity,
   *                 the method updates the last known estimates and sets flags
   *                 indicating new estimates are available.
   */
  public void setCurrentEstimates(AngularVelocity gyroRate) {
    PoseEstimate currentEstimateRight = new PoseEstimate();
    PoseEstimate currentEstimateLeft = new PoseEstimate();
    PoseEstimate currentEstimateBack = new PoseEstimate();

    if (useMegaTag2) {
      currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ConstVision.LIMELIGHT_RIGHT_NAME);
      currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ConstVision.LIMELIGHT_LEFT_NAME);
      currentEstimateBack = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ConstVision.LIMELIGHT_TOP_NAME);
    } else {
      currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue(ConstVision.LIMELIGHT_RIGHT_NAME);
      currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue(ConstVision.LIMELIGHT_LEFT_NAME);
      currentEstimateBack = LimelightHelpers.getBotPoseEstimate_wpiBlue(ConstVision.LIMELIGHT_TOP_NAME);
    }

    if (currentEstimateRight != null
        && !rejectUpdate(currentEstimateRight, gyroRate, ConstVision.AREA_THRESHOLD_FRONT)) {
      lastEstimateRight = currentEstimateRight;
      rightPose = currentEstimateRight.pose;
      newRightEstimate = true;
      rightTagCount = currentEstimateRight.tagCount;
    } else {
      rightTagCount = 0;
    }
    if (currentEstimateLeft != null && !rejectUpdate(currentEstimateLeft, gyroRate, ConstVision.AREA_THRESHOLD_FRONT)) {
      lastEstimateLeft = currentEstimateLeft;
      leftPose = currentEstimateLeft.pose;
      newLeftEstimate = true;
      leftTagCount = currentEstimateLeft.tagCount;
    } else {
      leftTagCount = 0;
    }
    if (currentEstimateBack != null && !rejectUpdate(currentEstimateBack, gyroRate, ConstVision.AREA_THRESHOLD_BACK)) {
      lastEstimateBack = currentEstimateBack;
      backPose = currentEstimateBack.pose;
      newBackEstimate = true;
      backTagCount = currentEstimateBack.tagCount;
    } else {
      backTagCount = 0;
    }
  }

  public boolean isVisionEnabled() {
    return visionEnabled;
  }

  public void setVisionEnabled(boolean enabled) {
    this.visionEnabled = enabled;
  }

  public Optional<PoseEstimate> determinePoseEstimate(AngularVelocity gyroRate) {
    setCurrentEstimates(gyroRate);

    // No valid pose estimates :(
    if (!newRightEstimate && !newLeftEstimate && !newBackEstimate) {
      limelightInUse = LL_INUSE.NONE.toString();
      return Optional.empty();

    } else if (newRightEstimate && !newLeftEstimate && !newBackEstimate) {
      // One valid pose estimate (right)
      limelightInUse = LL_INUSE.RIGHT.toString();
      newRightEstimate = false;
      return Optional.of(lastEstimateRight);

    } else if (!newRightEstimate && newLeftEstimate && !newBackEstimate) {
      // One valid pose estimate (left)
      limelightInUse = LL_INUSE.LEFT.toString();
      newLeftEstimate = false;
      return Optional.of(lastEstimateLeft);

    } else if (!newRightEstimate && !newLeftEstimate && newBackEstimate) {
      // One valid pose estimate (back)
      newBackEstimate = false;
      limelightInUse = LL_INUSE.BACK.toString();
      return Optional.of(lastEstimateBack);

    } else {
      // More than one valid pose estimate, use the closest one
      newRightEstimate = false;
      newLeftEstimate = false;
      newBackEstimate = false;
      if (rightTagCount >= leftTagCount
          && rightTagCount >= backTagCount) {
        limelightInUse = LL_INUSE.RIGHT.toString();
        return Optional.of(lastEstimateRight);
      } else if (leftTagCount > rightTagCount
          && leftTagCount > backTagCount) {
        limelightInUse = LL_INUSE.LEFT.toString();
        return Optional.of(lastEstimateLeft);
      } else if (backTagCount > rightTagCount
          && backTagCount > leftTagCount) {
        limelightInUse = LL_INUSE.BACK.toString();
        return Optional.of(lastEstimateBack);
      } else {
        limelightInUse = LL_INUSE.NONE.toString();
        return Optional.empty();
      }
    }
  }

  @Override
  public void periodic() {
  }
}