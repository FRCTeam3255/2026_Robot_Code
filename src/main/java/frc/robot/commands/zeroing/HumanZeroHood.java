// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.zeroing;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ConstMotion;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HumanZeroHood extends Command {
  /** Creates a new ManualZeroHood. */

  boolean zeroingSuccess = false;
  Time zeroingTimestamp = Units.Seconds.of(0);

  AngularVelocity lastRotorVelocity = Units.RotationsPerSecond.of(0);

  public HumanZeroHood() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.motionInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    zeroingSuccess = false;
    RobotContainer.motionInstance.hasHoodZeroed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if we have raised the hood above a certain speed
    if (RobotContainer.motionInstance.getHoodRotorVelocity().gte(ConstMotion.MANUAL_ZEROING_START_VELOCITY)
        || RobotContainer.motionInstance.hoodAttemptingZeroing) {
      // Enter zeroing mode!
      if (!RobotContainer.motionInstance.hoodAttemptingZeroing) {
        RobotContainer.motionInstance.hoodAttemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        System.out.println("Hood Zeroing Started!");
      }

      // Check if time elapsed is too high (zeroing timeout)
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(ConstMotion.ZEROING_TIMEOUT)) {
        RobotContainer.motionInstance.hoodAttemptingZeroing = false;
        System.out.println("Hood Zeroing Failed :(");
      } else {
        boolean deltaRotorVelocity = RobotContainer.motionInstance.getHoodRotorVelocity()
            .minus(lastRotorVelocity)
            .lte(ConstMotion.MANUAL_ZEROING_DELTA_VELOCITY);

        if (deltaRotorVelocity && lastRotorVelocity.lte(Units.RotationsPerSecond.of(0))) {
          zeroingSuccess = true;
        } else {
          lastRotorVelocity = RobotContainer.motionInstance.getHoodRotorVelocity();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (!interrupted && zeroingSuccess) {
      RobotContainer.motionInstance.hasHoodZeroed = true;
      RobotContainer.motionInstance.resetHoodSensorPosition(ConstMotion.ZEROED_HOOD_MANUAL_POS);
      System.out.println("Hood Zeroing Successful!!!! Yippee and hooray!!! :3");
    } else {
      System.out.println("Hood was never zeroed :((( blame eli");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return zeroingSuccess && RobotContainer.motionInstance.isHoodRotorVelocityZero();
  }
}
