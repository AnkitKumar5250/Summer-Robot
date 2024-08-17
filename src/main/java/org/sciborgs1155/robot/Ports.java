package org.sciborgs1155.robot;

import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.SwerveModulePorts;

/** Contains Port identifiers for hardware. */
public final class Ports {
  /** Ports for controllers. */
  public static final class Operator {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  /** Ports for drivetrain. */
  public static final class Drive {
    public static final SwerveModulePorts FRONT_LEFT = new SwerveModulePorts();
    public static final SwerveModulePorts FRONT_RIGHT = new SwerveModulePorts();
    public static final SwerveModulePorts REAR_LEFT = new SwerveModulePorts();
    public static final SwerveModulePorts REAR_RIGHT = new SwerveModulePorts();

    public static final int GYRO_ID = 0;
  }

  /** Ports for elevator. */
  public static final class Elevator {
    public static final int ELEVATOR_LEADER = 18;
    public static final int ELEVATOR_FOLLOWER = 19;

    public static final int ELEVATOR_ENCODER_PIN = 0;
  }
}
