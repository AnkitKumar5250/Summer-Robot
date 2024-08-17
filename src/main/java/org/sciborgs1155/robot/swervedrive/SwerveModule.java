package org.sciborgs1155.robot.swervedrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.DISTANCE_PER_ROTATION;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.RADIANS_PER_ROTATION;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.SwerveModulePorts;
import com.ctre.phoenix6.hardware.TalonFX;
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

  /** Encoder for translation motor. */
  private DutyCycleEncoder translationEncoder;

  /** Encoder for rotation motor. */
  private DutyCycleEncoder rotationEncoder;

  /**
   * Instantiates Swerve module.
   * 
   * @param ports : Configures the swerve module with hardware ID's.
   */
  public SwerveModule(SwerveModulePorts ports) {
    // Retrieves and assigns correct device ID's based on motor position.
    translation = ports.getTranslationMotor();
    rotation = ports.getRotationMotor();

    // Applies default hardware configuration to motors.
    translation.getConfigurator().apply(SwerveDriveConstants.defaultConfiguration);
    rotation.getConfigurator().apply(SwerveDriveConstants.defaultConfiguration);

    // Sets the distance per one rotation of the motors.
    translationEncoder.setDistancePerRotation(DISTANCE_PER_ROTATION.in(Meters));
    rotationEncoder.setDistancePerRotation(RADIANS_PER_ROTATION.in(Radians));
  }

  /**
   * Sets the voltage of the translational motor.
   * 
   * @param volts : target voltage.
   * @return Command.
   */
  public Command setTranslationVoltage(Measure<Voltage> volts) {
    return runOnce(() -> translation.setVoltage(volts.in(Volts)));
  }

  /**
   * Sets the voltage of the rotational motor.
   * 
   * @param volts : target voltage.
   * @return Command.
   */
  public Command setRotationVoltage(Measure<Voltage> volts) {
    return runOnce(() -> rotation.setVoltage(volts.in(Volts)));
  }


  /**
   * Returns the distance traveled(from encoder).
   * 
   * @return distance.
   */
  public Measure<Distance> getDistanceDisplacement() {
    return Meters.of(translationEncoder.getDistance());
  }

  /**
   * Returns the angle rotated(from encoder).
   * 
   * @return angle.
   */
  public Measure<Angle> getAngleDisplacement() {
    return Radians.of(rotationEncoder.getDistance());
  }

}
