// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs.controllerIDs;

public class ControllerMap extends SubsystemBase {
  private final SN_XboxController conDriver = new SN_XboxController(controllerIDs.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(controllerIDs.OPERATOR_USB);

  /** Creates a new ControllerMap. */
  public ControllerMap() {
  }

  public static void configDriverBindings() {
    // remember conDriver.(what button).on/while,True/False(Commands.runOnce(() ->
    // (what you want to do)));

  }

  public static void configOperatorBindings() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
