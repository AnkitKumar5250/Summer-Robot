package org.sciborgs1155.robot;

/**
 * Contains Port identifiers for hardware.
 */
public final class Ports {
  /**
   * Ports for controllers.
   */
  public static final class Operator {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  /**
   * Ports for drivetrain.
   */
  public static final class Drive {
    public static final int FRONT_LEFT_DRIVE = 11;
    public static final int REAR_LEFT_DRIVE = 10;
    public static final int FRONT_RIGHT_DRIVE = 12;
    public static final int REAR_RIGHT_DRIVE = 13;

    public static final int FRONT_LEFT_ENCODER_PIN = 15;
    public static final int REAR_LEFT_ENCODER_PIN = 14;
    public static final int FRONT_RIGHT_ENCODER_PIN = 16;
    public static final int REAR_RIGHT_ENCODER_PIN = 17;

    public static final int GYRO = 18;
  }

  /**
   * Ports for elevator.
   */
  public static final class Elevator {
    public static final int ELEVATOR_LEADER = 18;
    public static final int ELEVATOR_FOLLOWER = 19;

    public static final int ELEVATOR_ENCODER_PIN = 0;
  }
}
