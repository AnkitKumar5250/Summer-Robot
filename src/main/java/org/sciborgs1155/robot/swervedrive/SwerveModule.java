package org.sciborgs1155.robot.swervedrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.MAXIMUM_MOTOR_VOLTAGE;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.MINIMUM_MOTOR_VOLTAGE;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.TRACK_WIDTH;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.RotationFFD;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.SwerveModuleConfig;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.TranslationFFD;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.VoltagePID;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
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

  /** FFD controller used for translation(calculates voltage from velocity). */
  private SimpleMotorFeedforward translationFeedforward;

  /** FFD controller used for rotation(calculates voltage from velocity). */
  private SimpleMotorFeedforward rotationFeedforward;

  /** PID controller used for converting velocity setpoints to voltage outputs. */
  private PIDController voltagePID;

  /** Vector representing the target state of the module */
  private Translation2d targetVector;

  /**
   * Instantiates Swerve module.
   * 
   * @param config : Configures the swerve module with hardware ID's.
   */
  public SwerveModule(SwerveModuleConfig config) {
    // Instantiates Feedforward.
    translationFeedforward = TranslationFFD.getController();
    rotationFeedforward = RotationFFD.getController();

    // Instantiates PID.
    voltagePID = VoltagePID.getController();

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
    position = new SwerveModulePosition(translationEncoder.getDistance(),
        Rotation2d.fromRadians(rotationEncoder.getDistance()));
    location = config.getRelativeLocation();

  }

  /**
   * Sets the voltage of the Translation motor.
   * 
   * @param volts : target voltage.
   * @return Command.
   */
  private Command setTranslationVoltage(Measure<Voltage> volts) {
    // Clamps voltage value to be within certain limits.
    return runOnce(() -> translation.setVoltage(MathUtil.clamp(volts.in(Volts),
        MAXIMUM_MOTOR_VOLTAGE.in(Volts), MINIMUM_MOTOR_VOLTAGE.in(Volts))));
  }

  /**
   * Sets the voltage of the rotational motor.
   * 
   * @param volts : target voltage.
   * @return Command.
   */
  private Command setRotationVoltage(Measure<Voltage> volts) {
    // Clamps voltage value to be within certain limits.
    return runOnce(() -> rotation.setVoltage(MathUtil.clamp(volts.in(Volts),
        MAXIMUM_MOTOR_VOLTAGE.in(Volts), MINIMUM_MOTOR_VOLTAGE.in(Volts))));
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

  /** Sets the target vector. */
  public void setTargetVector(Translation2d targetVector) {
    this.targetVector = targetVector;
  }

  /** Applies vector setpoint to the module.*/
  public void apply() {
    
  }

}
