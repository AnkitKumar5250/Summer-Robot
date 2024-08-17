package org.sciborgs1155.robot.swervedrive;

import static org.sciborgs1155.robot.Ports.Drive.FRONT_LEFT;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_RIGHT;
import static org.sciborgs1155.robot.Ports.Drive.REAR_LEFT;
import static org.sciborgs1155.robot.Ports.Drive.REAR_RIGHT;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.RotationFFD;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.RotationPID;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.TranslationFFD;
import org.sciborgs1155.robot.swervedrive.SwerveDriveConstants.TranslationPID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Swerve drivetrain subsystem. */
public class SwerveDrive extends SubsystemBase {
  /** PID controller used for translaton. */
  private final PIDController pidControllerRotation = RotationPID.getController();

  /** PID controller used for rotation. */
  private final PIDController pidControllerTranslation = TranslationPID.getController();

  /** FFD controller used for translation. */
  private final SimpleMotorFeedforward translationFeedforward = TranslationFFD.getController();

  /** FFD controller used for rotation. */
  private final SimpleMotorFeedforward rotationFeedforward = RotationFFD.getController();

  /** Front left swerve module. */
  private final SwerveModule frontLeft = new SwerveModule(FRONT_LEFT);
  /** Front right swerve module. */
  private final SwerveModule frontRight = new SwerveModule(FRONT_RIGHT);
  /** Rear left swerve module. */
  private final SwerveModule rearLeft = new SwerveModule(REAR_LEFT);
  /** Rear right swerve module. */
  private final SwerveModule rearRight = new SwerveModule(REAR_RIGHT);


  /** Instantiates drivetrain. */
  public SwerveDrive() {}

}
