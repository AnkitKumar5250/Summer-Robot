package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * Constants for elevator subsystem.
 */
public class ElevatorConstants {
  // PID Controller constants
  public static final double kp = 1; // proportional
  public static final double ki = 0; // integral
  public static final double kd = 0; // derivative

  // FeedForward constants
  public static final double ks = 1; // static gain
  public static final double kg = 0; // gratvity gain
  public static final double kv = 0; // velocity gain
  public static final double ka = 1; // acceleration gain

  // Maximum acceptable error value in motor velocity
  public static final Measure<Velocity<Distance>> MINIMUM_VELOCITY_THRESHOLD =
      MetersPerSecond.of(0);

  // Distance traveled per one rotation of a wheel(in meters)
  public static final Measure<Distance> DISTANCE_PER_ROTATION = Meters.of(0);

  /**
   * Enumerator for describing certain elevation states.
   */
  enum Elevation {
    IDLE, GROUND, SMALL, LARGE,
  }
}
