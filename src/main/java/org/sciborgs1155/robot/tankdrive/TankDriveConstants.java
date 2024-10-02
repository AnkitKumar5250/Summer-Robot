package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/**
 * Constants for Tank drivetrain.
 */
public class TankDriveConstants {
  /** Default talon configuration. */
  public static final TalonFXConfiguration defaultTalonConfiguration = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  /** Default gyro configuration. */
  public static final Pigeon2Configuration defaultGyroConfiguration = new Pigeon2Configuration();

  // PID constants for rotation PID controller.

  /** Proportional component of the rotation PID controller. */
  public static final double kPR = 1;
  /** Integral component of the rotation PID controller. */
  public static final double kIR = 0.01;
  /** Derivative component of the rotation PID controller. */
  public static final double kDR = 0.1;

  // PID constants for drive PID controller.

  /** Proportional component of the drive PID controller. */
  public static final double kPD = 1;
  /** Integral component of the drive PID controller. */
  public static final double kID = 0.01;
  /** Derivative component of the drive PID controller. */
  public static final double kDD = 0.1;

  // FFD constants for drive.

  /** Linear velocity component of the FFD controller. */
  public static final double kLV = 1;
  /** Linear acceleration component of the FFD controller. */
  public static final double kLA = 0.1;
  /** Linear static component of the FFD controller. */
  public static final double kLS = 0.2;

  // FFD constants for rotation.

  /** Angular velocity component of the FFD controller. */
  public static final double kAV = 1;
  /** Angular acceleration component of the FFD controller. */
  public static final double kAA = 0.1;
  /** Angular static component of the FFD controller. */
  public static final double kAS = 0.2;

  /** Maximum voltage applicable(positive or negative). */
  public static final Measure<Voltage> MAXIMUM_VOLTAGE_MAGNITUDE = Volts.of(24);

  /** Max velocity of a motor. */
  public static final Measure<Velocity<Distance>> MAX_VELOCITY = MetersPerSecond.of(0);

  /** Max acceleration of a motor. */
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(0);

  /** Gearing ratio. */
  public static final double GEARING_RATIO = 7.29; // 7.29:1 gearing reduction.

  /** Moment of inertia. */
  public static final double MOMENT_OF_INERTIA = 7.5; // 7.5 kg m^2.

  /** Robot mass. */
  public static final Measure<Mass> ROBOT_MASS = Kilograms.of(60);

  /** Radius of each wheel. */
  public static final Measure<Distance> WHEEL_RADIUS = Inches.of(3); 

  /** Standard measurement deviations. */

  // The standard deviations for measurement noise:
  // x and y: 0.001 m
  // heading: 0.001 rad
  // l and r velocity: 0.1 m/s
  // l and r position: 0.005 m

  public static final Vector<N7> STANDARD_DEVIATIONS =
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);

  /** Maximum acceptable error value in distance. */
  public static final Measure<Distance> MINIMUM_DISTANCE_THRESHOLD = Meters.of(0.01);

  /** Maximum acceptable error value in angle. */
  public static final Measure<Angle> MINIMUM_DEGREE_THRESHOLD = Degrees.of(0.1);

  /** Distance between the outer edges of the left and right wheels. */
  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0.7112);

  /** Initial robot pose. */
  public static final Pose2d INITIAL_ROBOT_POSE = new Pose2d();

}
