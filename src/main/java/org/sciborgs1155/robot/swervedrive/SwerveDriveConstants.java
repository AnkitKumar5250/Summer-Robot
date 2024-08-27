package org.sciborgs1155.robot.swervedrive;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Constants for swerve drivetrain.
 */
public class SwerveDriveConstants {
  /**
   * PID constants for translational motion.
   */
  public static final class TranslationPID {
    /** Proportional coefficient */
    public static final double P = 0;
    /** Integral coefficient */
    public static final double I = 0;
    /** Derivative coefficient */
    public static final double D = 0;

    /**
     * Returns an instance of the controller with tuned constants.
     * 
     * @return PID controller with tuned constants.
     */
    public static final PIDController getController() {
      PIDController controller = new PIDController(P, I, D);
      controller.setTolerance(DISTANCE_TOLERANCE.in(Meters));
      return controller;
    }
  }

  /**
   * PID constants for rotational motion.
   */
  public static final class RotationPID {
    /** Proportional coefficient */
    public static final double P = 0;
    /** Integral coefficient */
    public static final double I = 0;
    /** Derivative coefficient */
    public static final double D = 0;

    /**
     * Returns an instance of the controller with tuned constants.
     * 
     * @return PID controller with tuned constants.
     */
    public static final PIDController getController() {
      PIDController controller = new PIDController(P, I, D);
      controller.setTolerance(ANGLE_TOLERANCE.in(Radians));
      return controller;
    }
  }

  /**
   * PID constants for calculating voltage from velocity setpoint.
   */
  public static final class VoltagePID {
    /** Proportional coefficient */
    public static final double P = 0;
    /** Integral coefficient */
    public static final double I = 0;
    /** Derivative coefficient */
    public static final double D = 0;

    /**
     * Returns an instance of the controller with tuned constants.
     * 
     * @return PID controller with tuned constants.
     */
    public static final PIDController getController() {
      PIDController controller = new PIDController(P, I, D);
      controller.setTolerance(ANGLE_TOLERANCE.in(Radians));
      return controller;
    }
  }

  /**
   * FFD constants for translational motion.
   */
  public static final class TranslationFFD {
    /** Static gain. */
    public static final double S = 0;
    /** Velocity gain. */
    public static final double V = 0;
    /** Acceleration gain. */
    public static final double A = 0;

    /**
     * Returns an instance of the controller with tuned constants.
     * 
     * @return FFD controller with tuned constants.
     */
    public static final SimpleMotorFeedforward getController() {
      return new SimpleMotorFeedforward(S, V, A);
    }
  }

  /**
   * FFD constants for rotational motion.
   */
  public static final class RotationFFD {
    /** Static gain. */
    public static final double S = 0;
    /** Velocity gain. */
    public static final double V = 0;
    /** Acceleration gain. */
    public static final double A = 0;

    /**
     * Returns an instance of the controller with tuned constants.
     * 
     * @return FFD controller with tuned constants.
     */
    public static final SimpleMotorFeedforward getController() {
      return new SimpleMotorFeedforward(S, V, A);
    }
  }

  /** Maximum voltage attainable by the motors in the drivetrain. */
  public static final Measure<Voltage> MAXIMUM_MOTOR_VOLTAGE = Volts.of(0);

  /** Minimum voltage attainable by the motors in the drivetrain. */
  public static final Measure<Voltage> MINIMUM_MOTOR_VOLTAGE = Volts.of(0);

  /** Default motor configuration(but motors brake instead of coast) */
  public static final TalonFXConfiguration defaultConfiguration = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  /** Maximum acceptable error in distance(for PID controllers) */
  public static final Measure<Distance> DISTANCE_TOLERANCE = Meters.of(0);

  /** Maximum acceptable error in angle(for PID controllers) */
  public static final Measure<Angle> ANGLE_TOLERANCE = Radians.of(0);

  /** Refers to the position of the module on the drivetrain. Used for initializing modules. */
  public static enum ModulePosition {
    FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
  }

  /** Interface for swerve module configuration. */
  public static class SwerveModuleConfig {
    /** ID of the translation motor. */
    public int TRANSLATION_MOTOR_ID = 0;

    /** ID of the rotation motor. */
    public int ROTATION_MOTOR_ID = 0;

    /** Encoder pin of the translation encoder. */
    public int TRANSLATION_ENCODER_PIN = 0;

    /** Encoder pin of the rotation encoder. */
    public int ROTATION_ENCODER_PIN = 0;

    /** X coordinate of the module relative to robot orgin. */
    public Measure<Distance> xLocation;

    /** Y coordinate of the module relative to robot orgin. */
    public Measure<Distance> ylocation;

    /** Unit vector for rotational movement of the robot */
    public Translation2d rotationalUnitVector;

    /** Returns an instance of the translation motor. */
    public TalonFX getTranslationMotor() {
      return new TalonFX(TRANSLATION_MOTOR_ID);
    }

    /** Returns an instance of the translation motor. */
    public TalonFX getRotationMotor() {
      return new TalonFX(ROTATION_MOTOR_ID);
    }

    /** Returns an instance of the translation motor encoder. */
    public DutyCycleEncoder getTranslationEncoder() {
      return new DutyCycleEncoder(TRANSLATION_ENCODER_PIN);
    }

    /** Returns an instance of the rotation motor encoder. */
    public DutyCycleEncoder getRotationEncoder() {
      return new DutyCycleEncoder(ROTATION_ENCODER_PIN);
    }

    /** Returns a Translation2D representing the modules position relative to the orgin. */
    public Translation2d getRelativeLocation() {
      return new Translation2d(xLocation, ylocation);
    }

    /**
     * Returns unit vector for rotational movement of the robot.
     * 
     * @return vector.
     */
    public Translation2d getRotationalUnitVector() {
      return rotationalUnitVector;
    }


    /** Instantiates constants. */
    public SwerveModuleConfig() {

    }
  }

  /** Distance between the outer edges of the left and right wheels */
  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0);

  /** Initial robot pose */
  public static final Pose2d INITIAL_ROBOT_POSE = new Pose2d();

}
