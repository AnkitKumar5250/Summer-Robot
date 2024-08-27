package org.sciborgs1155.robot.swervedrive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.INITIAL_ROBOT_POSE;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.RotationPID;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.TranslationPID;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Swerve drivetrain subsystem. */
public class SwerveDrive extends SubsystemBase {
  /** PID controller used for translaton(calculates velocity from angle). */
  private PIDController pidControllerRotation;

  /** PID controller used for rotation(calculates velocity from distance). */
  private PIDController pidControllerTranslation;

  /** Pose setpoint */
  private Pose2d setpoint;

  /** Gyroscope used to track rotation of the robot. */
  private Pigeon2 gyroscope = new Pigeon2(GYRO_ID);

  /** Front left swerve module. */
  private SwerveModule frontLeft = new SwerveModule(FRONT_LEFT);
  /** Front right swerve module. */
  private SwerveModule frontRight = new SwerveModule(FRONT_RIGHT);
  /** Rear left swerve module. */
  private SwerveModule rearLeft = new SwerveModule(REAR_LEFT);
  /** Rear right swerve module. */
  private SwerveModule rearRight = new SwerveModule(REAR_RIGHT);

  /** Kinematics of the drivetrain */
  SwerveDriveKinematics kinematics;

  /** Tracks the current position and orientation of the robot. */
  private SwerveDrivePoseEstimator poseEstimator;

  /**
   * Updates motor voltages based on commanded setpoints, and updates odometry measurements.
   * 
   * @param visionPoseEstimate : Current vision measurements.
   */
  public void updateOdometry(Pose2d visionPoseEstimate) {
    // Updates Odometry.
    poseEstimator.addVisionMeasurement(visionPoseEstimate, PERIOD.in(Seconds));
    poseEstimator.updateWithTime(PERIOD.in(Seconds), gyroscope.getRotation2d(),
        new SwerveModulePosition[] {frontLeft.getAbsolutePosition(),
            frontRight.getAbsolutePosition(), rearLeft.getAbsolutePosition(),
            rearRight.getAbsolutePosition()});
  }

  /*
   * Instantiates drivetrain.
   * 
   * @return Instance of drivetrain.
   */
  public SwerveDrive() {
    /** PID controller used for translaton(calculates velocity from angle). */
    pidControllerRotation = RotationPID.getController();
    pidControllerRotation.setSetpoint(0);

    /** PID controller used for rotation(calculates velocity from distance). */
    pidControllerTranslation = TranslationPID.getController();
    pidControllerTranslation.setSetpoint(0);

    /** Tracks the current position and orientation of the robot. */
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroscope.getRotation2d(),
        new SwerveModulePosition[] {frontLeft.getAbsolutePosition(),
            frontRight.getAbsolutePosition(), rearLeft.getAbsolutePosition(),
            rearRight.getAbsolutePosition()},
        INITIAL_ROBOT_POSE);

    /** Kinematics of the drivetrain */
    kinematics =
        new SwerveDriveKinematics(frontLeft.getRelativeLocation(), frontRight.getRelativeLocation(),
            rearLeft.getRelativeLocation(), rearRight.getRelativeLocation());
  }

  /**
   * Current orientation of the robot(used for PID).
   * 
   * @return error
   */
  public Measure<Angle> getRotationalMeasurement() {
    return Radians.of(poseEstimator.getEstimatedPosition().getRotation()
        .minus(setpoint.getRotation()).getRadians());
  }

  /**
   * Current distance of the robot(used for PID).
   * 
   * @return error
   */
  public Measure<Distance> getTranslationalMeasurement() {
    return Meters.of(poseEstimator.getEstimatedPosition().getTranslation()
        .getDistance(setpoint.getTranslation()));
  }

  /**
   * Calculates the target angular velocity of the drivetrain.
   * 
   * @return Target angular velocity.
   */
  public Measure<Velocity<Angle>> calcTargetAngularVelocity() {
    // Uses PID to calculated the target angular velocity.
    return RadiansPerSecond
        .of(pidControllerRotation.calculate(getRotationalMeasurement().in(Radians)));
  }

  /**
   * Calculates the target translational velocity of the drivetrain.
   * 
   * @return Target translational velocity.
   */
  public Measure<Velocity<Distance>> calcTargetTranslationalVelocity() {
    // Uses PID to calculated the target translational velocity.
    return MetersPerSecond
        .of(pidControllerTranslation.calculate(getTranslationalMeasurement().in(Meters)));
  }

  /** Merges translational and angular motion vectors into one uniform vector for motion, and . */
  public Translation2d[] getMotionVector() {
    Translation2d frontLeftVector = FRONT_LEFT.getRotationalUnitVector().times(calcTargetAngularVelocity().in(RadiansPerSecond));
    return null;
  }
}
