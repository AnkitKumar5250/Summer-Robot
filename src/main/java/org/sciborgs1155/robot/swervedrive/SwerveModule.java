package org.sciborgs1155.robot.swervedrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.MAXIMUM_MOTOR_VOLTAGE;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.MINIMUM_MOTOR_VOLTAGE;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.TRACK_WIDTH;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.SwerveModuleConfig;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A module containing a rotational and translational motor. */
public class SwerveModule extends SubsystemBase {
  /** Motor used for translation. */
  private TalonFX translation;

  /** Motor used for rotation. */
  private TalonFX rotation;

  /** Location of the swerve drive module relative to the robot center. */
  private Translation2d location;

  /** Absolute position of the swerve module(orientation and distance traveled). */
  private SwerveModulePosition position;

  /** Encoder for translation motor. */
  private DutyCycleEncoder translationEncoder;

  /** Encoder for rotation motor. */
  private DutyCycleEncoder rotationEncoder;



  /**
   * Instantiates Swerve module.
   * 
   * @param config : Configures the swerve module with hardware ID's.
   */
  public SwerveModule(SwerveModuleConfig config) {
    // Retrieves and assigns correct device ID's based on motor position.
    translation = config.getTranslationMotor();
    rotation = config.getRotationMotor();

    // Applies default hardware configuration to motors.
    translation.getConfigurator().apply(SwerveDriveConstants.defaultConfiguration);
    rotation.getConfigurator().apply(SwerveDriveConstants.defaultConfiguration);

    // Sets the distance per one rotation of the motors.
    translationEncoder.setDistancePerRotation(TRACK_WIDTH.in(Meters) * Math.PI * 2);
    rotationEncoder.setDistancePerRotation(Math.PI * 2);

    // Instantiates position and location.
    position = new SwerveModulePosition(getDistanceEncoder(),
        Rotation2d.fromRadians(getAngleEncoder().in(Radians)));
    location = config.getRelativeLocation();
  }

  /**
   * Sets the voltage of the translational motor.
   * 
   * @param volts : target voltage.
   * @return Command.
   */
  public Command setTranslationVoltage(Measure<Voltage> volts) {
    // Clamps voltage value to be within certain limits.
    if (volts.gt(MAXIMUM_MOTOR_VOLTAGE)) {
      volts = MAXIMUM_MOTOR_VOLTAGE;
    }
    if (volts.lt(MINIMUM_MOTOR_VOLTAGE)) {
      volts = MINIMUM_MOTOR_VOLTAGE;
    }

    // Clamped voltage value.
    final Measure<Voltage> cvolts = volts;

    // Returns a command that sets the motor voltage.
    return runOnce(() -> translation.setVoltage(cvolts.in(Volts)));
  }

  /**
   * Sets the voltage of the rotational motor.
   * 
   * @param volts : target voltage.
   * @return Command.
   */
  public Command setRotationVoltage(Measure<Voltage> volts) {
    // Clamps voltage value to be within certain limits.
    if (volts.gt(MAXIMUM_MOTOR_VOLTAGE)) {
      volts = MAXIMUM_MOTOR_VOLTAGE;
    }
    if (volts.lt(MINIMUM_MOTOR_VOLTAGE)) {
      volts = MINIMUM_MOTOR_VOLTAGE;
    }

    // Clamped voltage value.
    final Measure<Voltage> cvolts = volts;

    // Returns a command that sets the motor voltage.
    return runOnce(() -> rotation.setVoltage(cvolts.in(Volts)));
  }


  /**
   * Returns the distance traveled(from encoder).
   * 
   * @return distance.
   */
  public Measure<Distance> getDistanceEncoder() {
    return Meters.of(translationEncoder.getDistance());
  }

  /**
   * Returns the angle rotated(from encoder).
   * 
   * @return angle.
   */
  public Measure<Angle> getAngleEncoder() {
    return Radians.of(rotationEncoder.getDistance());
  }

  /**
   * Returns the relative location of the module compared to the orgin of the robot.
   * 
   * @return Translation representing a coordinate with the robot orgin being (0,0)
   */
  public Translation2d getRelativeLocation() {
    return location;
  }

  /**
   * Returns the absolute location and orientation of the module compared to the original position
   * it was in.
   * 
   * @return position.
   */
  public SwerveModulePosition getAbsolutePosition() {
    return position;
  }

  /**
   * Converts robot-relative vector to a module-relative vector.
   * @param absoluteVector : vector representing a velocity and direction for a module relative to the robot.
   * @return vector representing a velocity and direction relative to the module.
   */
  public Translation2d getRelativeVector(Translation2d absoluteVector) {
    return absoluteVector;
  }
}
