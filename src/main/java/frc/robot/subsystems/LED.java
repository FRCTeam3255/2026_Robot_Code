// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs.LEDIDs;
import frc.robot.constants.ConstLEDs;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.hardware.CANdle;

/**
 * Subsystem that controls an addressable LED strip using a CANdle.
 */
public class LED extends SubsystemBase {
  // TODO: ADD CAN BUS
    private final CANBus kCANBus = new CANBus("");
    private final CANdle m_candle = new CANdle(LEDIDs.LED_CAN, kCANBus);
    private ColorFlowAnimation LEDAnimation = new ColorFlowAnimation(0, 0);

    public LED() {
        setDefaultCommand(updateLEDs(LEDAnimation));
    }

    /**
     * Updates the animations and LEDs of the CANdle.
     *
     * @return Command to run
     */
    public Command updateLEDs(ColorFlowAnimation slot) {
        return run(() -> {
            m_candle.setControl(slot);
        });
    }

    public void setLEDAnimation(ColorFlowAnimation slot){
      LEDAnimation = slot;
    }
}
