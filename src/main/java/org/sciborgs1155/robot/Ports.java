package org.sciborgs1155.robot;

import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.SwerveModuleConfig;

/** Contains Port identifiers for hardware. */
public final class Ports {
  /** Ports for controllers. */
  public static final class Operator {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  /** Ports for drivetrain. */
  public static final class Drive {
    public static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig();
    public static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig();
    public static final SwerveModuleConfig REAR_LEFT = new SwerveModuleConfig();
    public static final SwerveModuleConfig REAR_RIGHT = new SwerveModuleConfig();

    public static final int GYRO_ID = 0;
  }

  /** Ports for elevator. */
  public static final class Elevator {
    public static final int ELEVATOR_LEADER = 18;
    public static final int ELEVATOR_FOLLOWER = 19;

    public static final int ELEVATOR_ENCODER_PIN = 0;
  }
}
