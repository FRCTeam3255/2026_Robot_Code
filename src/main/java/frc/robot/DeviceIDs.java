package frc.robot;

import com.ctre.phoenix6.CANBus;

// Contains all ports on our robot

public class DeviceIDs {
  public static final int PRAC_BOT_DIO = 0;

  public static class controllerIDs {
    public static final int DRIVER_USB = 0;
    public static final int OPERATOR_USB = 1;
  }

  // Drivetrain IDs: 0~9
  public static class drivetrainIDs {
    public static final CANBus CAN_BUS_NAME = new CANBus("Swerve");
    public static final int PIGEON_CAN = 0;

    // Module 0
    public static final int FRONT_LEFT_DRIVE_CAN = 0;
    public static final int FRONT_LEFT_STEER_CAN = 1;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_CAN = 0;

    // Module 1
    public static final int FRONT_RIGHT_DRIVE_CAN = 2;
    public static final int FRONT_RIGHT_STEER_CAN = 3;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_CAN = 1;

    // Module 2
    public static final int BACK_LEFT_DRIVE_CAN = 4;
    public static final int BACK_LEFT_STEER_CAN = 5;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_CAN = 2;

    // Module 3
    public static final int BACK_RIGHT_DRIVE_CAN = 6;
    public static final int BACK_RIGHT_STEER_CAN = 7;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_CAN = 3;
  }

  // Rotor and Motion IDs: 10~29
  public static class rotorIDs {
    public static final int SERIALIZER_ROLLERS_CAN = 10;
    public static final int INTAKE_ROLLERS_CAN = 11;
    public static final int SHOOTER_TRANSFER_CAN = 12;
    public static final int SERIALIZER_V_FUNNEL_CAN = 13;
    public static final int FLYWHEEL_RIGHT_CAN = 14;
    public static final int FLYWHEEL_LEFT_CAN = 15;
  }

  // Motion Profile IDs: 30~49
  public static class motionIDs {
    public static final int INTAKE_PIVOT_CAN = 30;
    public static final int CLIMBER_CAN = 31;
    public static final int HOOD_CAN = 32;
  }
}