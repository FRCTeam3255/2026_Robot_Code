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
public class ManualZeroIntake extends Command {

  boolean zeroingSuccess = false;
  Time zeroingTimestamp = Units.Seconds.of(0);

  AngularVelocity lastRotorVelocity = Units.RotationsPerSecond.of(0);

  /** Creates a new ManualZeroIntake. */
  public ManualZeroIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    zeroingSuccess = false;
    RobotContainer.motionInstance.hasIntakePivotZeroed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if we have raised the intake above a certain speed
    if (RobotContainer.motionInstance.getIntakePivotRotorVelocity().gte(ConstMotion.MANUAL_ZEROING_START_VELOCITY)
        || RobotContainer.motionInstance.intakePivotAttemptingZeroing) {
      // Enter zeroing mode!
      if (!RobotContainer.motionInstance.intakePivotAttemptingZeroing) {
        RobotContainer.motionInstance.intakePivotAttemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        System.out.println("Intake Pivot Zeroing Started!");
      }

      // Check if time elapsed is too high (zeroing timeout)
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(ConstMotion.ZEROING_TIMEOUT)) {
        RobotContainer.motionInstance.intakePivotAttemptingZeroing = false;
        System.out.println("Intake Pivot Zeroing Failed :(");
      } else {
        boolean deltaRotorVelocity = RobotContainer.motionInstance.getIntakePivotRotorVelocity()
            .minus(lastRotorVelocity)
            .lte(ConstMotion.MANUAL_ZEROING_DELTA_VELOCITY);

        if (deltaRotorVelocity && lastRotorVelocity.lte(Units.RotationsPerSecond.of(0))) {
          zeroingSuccess = true;
        } else {
          lastRotorVelocity = RobotContainer.motionInstance.getIntakePivotRotorVelocity();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.motionInstance.setIntakePivotSoftwareLimits(true, true);

    if (!interrupted && zeroingSuccess) {
      RobotContainer.motionInstance.hasIntakePivotZeroed = true;
      RobotContainer.motionInstance.resetIntakePivotSensorPosition(ConstMotion.ZEROED_INTAKE_MANUAL_POS);
      System.out.println("Intake Pivot Zeroing Successful!!!! Yippee and hooray!!! :3");
    } else {
      System.out.println("Intake Pivot was never zeroed :((( blame eli");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return zeroingSuccess && RobotContainer.motionInstance.isIntakePivotRotorVelocityZero();
  }
}
