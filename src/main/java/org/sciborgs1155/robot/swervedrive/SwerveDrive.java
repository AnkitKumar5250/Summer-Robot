package org.sciborgs1155.robot.swervedrive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.INITIAL_ROBOT_POSE;
import static org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.TRACK_WIDTH;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.RotationFFD;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.RotationPID;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.TranslationFFD;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.TranslationPID;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.VoltagePID;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private final PIDController pidControllerRotation = RotationPID.getController();

  /** PID controller used for rotation(calculates velocity from distance). */
  private final PIDController pidControllerTranslation = TranslationPID.getController();

  /** PID controller used for voltage calculations(calculates voltage from velocity). */
  private final PIDController pidControllerVoltage = VoltagePID.getController();

  /** FFD controller used for translation(calculates voltage from velocity). */
  private final SimpleMotorFeedforward translationFeedforward = TranslationFFD.getController();

  /** FFD controller used for rotation(calculates voltage from velocity). */
  private final SimpleMotorFeedforward rotationFeedforward = RotationFFD.getController();

  /** Gyroscope used to track rotation of the robot. */
  private final Pigeon2 gyroscope = new Pigeon2(GYRO_ID);

  /** Front left swerve module. */
  private final SwerveModule frontLeft = new SwerveModule(FRONT_LEFT);
  /** Front right swerve module. */
  private final SwerveModule frontRight = new SwerveModule(FRONT_RIGHT);
  /** Rear left swerve module. */
  private final SwerveModule rearLeft = new SwerveModule(REAR_LEFT);
  /** Rear right swerve module. */
  private final SwerveModule rearRight = new SwerveModule(REAR_RIGHT);

  /**
   * Returns the absolute positions of all the swerve modules.
   * 
   * @return An array of swerve module positions.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {frontLeft.getAbsolutePosition(),
        frontRight.getAbsolutePosition(), rearLeft.getAbsolutePosition(),
        rearRight.getAbsolutePosition()};
  }

  /** Kinematics of the drivetrain */
  SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(frontLeft.getRelativeLocation(), frontRight.getRelativeLocation(),
          rearLeft.getRelativeLocation(), rearRight.getRelativeLocation());

  /** Tracks the current position and orientation of the robot. */
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics,
      gyroscope.getRotation2d(), getModulePositions(), INITIAL_ROBOT_POSE);

  /** Updates motor voltages based on commanded setpoints, and updates odometry measurements. */
  public void update() {
    // Updates Odometry.
    poseEstimator.update(gyroscope.getRotation2d(), getModulePositions());
  }

  /**
   * Calculates a module relative vector representing a target path for a module.
   * @return A module relative vector.
   */
  public Translation2d calcRotationVector() {
    // Current orientation of the robot.
    Measure<Angle> measuredAngle =
        Radians.of(poseEstimator.getEstimatedPosition().getRotation().getRadians());

    // Uses PID to calculated the target angular velocity.
    Measure<Velocity<Angle>> targetAnglularVelocity =
        RadiansPerSecond.of(pidControllerRotation.calculate(measuredAngle.in(Radians)));

    // Converts angular velocity to linear velocity(expressed as distance per second).
    Measure<Velocity<Distance>> targetLinearVelocity = MetersPerSecond
        .of(targetAnglularVelocity.in(RadiansPerSecond) * (TRACK_WIDTH.in(Meters) / 2));

    // Creates a unit vector representing the ideal (relative)orientation and velocity.
    Translation2d vector = new Translation2d(targetLinearVelocity.in(MetersPerSecond),
        Rotation2d.fromRadians(Math.PI / 4));

    return vector;
  }

  /** Instantiates drivetrain. */
  public SwerveDrive() {}

}
