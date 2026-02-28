// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.zeroing;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
// Timer is not used directly here; Motion.checkZeroing handles timing.
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstMotion;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroHood extends Command {
  Time zeroingTimestamp;
  boolean hasZeroed = false;

  /** Creates a new ZeroHoodPivot. */
  public ZeroHood() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.motionInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.motionInstance.setHoodSoftwareLimits(true, false);

    RobotContainer.motionInstance.setHoodVoltage(Units.Volts.zero());
    zeroingTimestamp = Units.Seconds.zero();
    hasZeroed = RobotContainer.motionInstance.hasHoodZeroed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.motionInstance.setHoodVoltage(ConstMotion.ZEROING_HOOD_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.motionInstance.setHoodSoftwareLimits(true, true);

    // Stop all movement
    RobotContainer.motionInstance.setHoodVoltage(Units.Volts.zero());

    // Reset to the current position if this command was not interrupted
    if (!interrupted) {
      RobotContainer.motionInstance.resetHoodSensorPosition(ConstMotion.ZEROED_HOOD_AUTO_POS);
      RobotContainer.motionInstance.hasHoodZeroed = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (hasZeroed) {
      return true;
    }

    // Use the Motion helper to perform the common zeroing check.
    frc.robot.subsystems.Motion.ZeroingResult res = RobotContainer.motionInstance
        .checkZeroing(RobotContainer.motionInstance.getHoodRotorVelocity(), zeroingTimestamp);
    // Persist updated timestamp and return finished state
    zeroingTimestamp = res.timestamp;
    return res.finished;
  }
}
