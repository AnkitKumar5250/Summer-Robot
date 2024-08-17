package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

/**
 * Constants for Tank drivetrain.
 */
public class TankDriveConstants {
  // Turning radius of the robot
  public static final Measure<Distance> TURNING_RADIUS = Meters.of(1);

  // Distance traveled per one rotation of a wheel(in meters)
  public static final Measure<Distance> DISTANCE_PER_ROTATION = Meters.of(1);

  // PID constants for rotation PID controller
  public static final double kPR = 1; // proportional
  public static final double kIR = 1; // integral
  public static final double kDR = 1; // derivative

  // PID constants for drive PID controller
  public static final double kPD = 1; // proportional
  public static final double kID = 1; // integral
  public static final double kDD = 1; // derivative

  // Maximum acceptable error value in motion
  public static final Measure<Distance> MINIMUM_DISTANCE_THRESHOLD = Meters.of(0.01);
  public static final Measure<Angle> MINIMUM_DEGREE_THRESHOLD = Degrees.of(0.1);

  // Distance between the outer edges of the left and right wheels
  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0);

  // Initial robot pose
  public static final Pose2d INITIAL_ROBOT_POSE = new Pose2d();

}
