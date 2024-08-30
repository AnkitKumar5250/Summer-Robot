package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_RIGHT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.GYRO;
import static org.sciborgs1155.robot.Ports.Drive.REAR_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_RIGHT_DRIVE;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.INITIAL_ROBOT_POSE;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.MAX_ACCELERATION;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.MINIMUM_DEGREE_THRESHOLD;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.MINIMUM_DISTANCE_THRESHOLD;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.TRACK_WIDTH;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.defaultGyroConfiguration;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kAA;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kAS;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kAV;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kDD;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kDR;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kID;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kIR;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kLA;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kLS;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kLV;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kPD;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.kPR;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Tank drivetrain Subsystem.
 */
public class TankDrive extends SubsystemBase {
  /** TankIO */
  private TankIO motors;

  /** Gyroscope */
  private Pigeon2 gyro;

  /** Used for inputting voltages into TankIO. */
  private DifferentialDrive differentialDrive;

  /** PID controller for translational motion. */
  private final ProfiledPIDController pidControllerDrive =
      new ProfiledPIDController(kPD, kID, kDD, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));

  /** PID controller for rotational motion. */
  private final ProfiledPIDController pidControllerRotation =
      new ProfiledPIDController(kPR, kIR, kDR, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));

  /** PID controller for voltage. */
  private final ProfiledPIDController pidControllerVoltage =
      new ProfiledPIDController(kPR, kIR, kDR, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));

  /** FFD controller for translational motion. */
  private final SimpleMotorFeedforward feedforwardDrive = new SimpleMotorFeedforward(kLS, kLV, kLA);

  /** FFD controller rotational motion. */
  private final SimpleMotorFeedforward feedforwardRotation =
      new SimpleMotorFeedforward(kAS, kAV, kAA);

  /** Kinematics of the drivetrain. */
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(TRACK_WIDTH);

  /** Used to track robot position. */
  private final DifferentialDrivePoseEstimator differentialDriveOdometry =
      new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), 0, 0,
          INITIAL_ROBOT_POSE);

  public static TankDrive sim() {
    return new TankDrive(
        SimTank.create(FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE, FRONT_RIGHT_DRIVE, REAR_RIGHT_DRIVE),
        false);
  }

  public static TankDrive fake() {
    return new TankDrive(
        FakeTank.create(FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE, FRONT_RIGHT_DRIVE, REAR_RIGHT_DRIVE),
        false);
  }

  public static TankDrive real() {
    return new TankDrive(
        RealTank.create(FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE, FRONT_RIGHT_DRIVE, REAR_RIGHT_DRIVE),
        true);
  }

  /**
   * Instantiates drivetrain.
   */
  private TankDrive(TankIO motors, boolean isReal) {
    // Instantiates everything.
    if (isReal) {
      this.gyro = new Pigeon2(GYRO);
    }
    this.motors = motors;

    this.differentialDrive = new DifferentialDrive(motors::setLeftVoltage, motors::setRightVoltage);

    // Applies default gyro configuration.
    gyro.getConfigurator().apply(defaultGyroConfiguration);

    // Sets the right to be inverted
    motors.setRightInverted(true);

    // Sets the tolerance of pid controllers
    pidControllerDrive.setTolerance(MINIMUM_DISTANCE_THRESHOLD.in(Meters));
    pidControllerRotation.setTolerance(MINIMUM_DEGREE_THRESHOLD.in(Degrees));

    // Sets the default command to stop
    setDefaultCommand(motors.defaultCommand());
  }

  /**
   * Gets the robots current position on the field.
   * 
   * @return The position of the robot.
   */
  public Pose2d getRobotPose() {
    return differentialDriveOdometry.getEstimatedPosition();
  }

  /**
   * Gets the robots current position on the field.
   * 
   * @return The position of the robot.
   */
  public Translation2d getRobotPosition() {
    return differentialDriveOdometry.getEstimatedPosition().getTranslation();
  }

  /**
   * Gets the robots current orientation.
   * 
   * @return The orientation of the robot(counterclockwise).
   */
  public Measure<Angle> getRobotOrientation() {
    return Degrees.of(gyro.getAngle());
  }

  /**
   * Returns distance traveled by the drivetrain(accounts for backwards travel).
   * 
   * @return Distance.
   */
  private Measure<Distance> getPosition() {
    return motors.getLeftDistance().plus(motors.getRightDistance()).divide(2);
  }

  private Command updateRobotPose() {
    return runOnce(() -> differentialDriveOdometry.update(gyro.getRotation2d(),
        motors.getLeftDistance().in(Meters), motors.getRightDistance().in(Meters)));
  }

  /**
   * Stops the drivetrain.
   * 
   * @return A command.
   */
  public Command stop() {
    return runOnce(() -> motors.stop()).andThen(runOnce(() -> updateRobotPose()));
  }

  /**
   * Drives based on driver input.
   * 
   * @param leftY : Y value of left joystick.
   * @param rightY : y value of right joystick.
   * @return A command.
   */
  public Command drive(double leftY, double rightY) {
    return run(() -> differentialDrive.tankDrive(leftY, rightY)).alongWith(updateRobotPose());
  }

  /**
   * Drives the robot a certain distance.
   * 
   * @param distance : distance to move the robot.
   * @return A command.
   */
  public Command drive(Measure<Distance> distance) {
    // Sets the velocity PID controllers setpoint.
    pidControllerDrive.setGoal(getPosition().in(Meters) + distance.in(Meters));
    return run(() -> {
      // Calculates velocity setpoint.
      Measure<Velocity<Distance>> velocitySetpoint =
          MetersPerSecond.of(pidControllerDrive.calculate(getPosition().in(Meters)));

      // Calculates voltage outputs based on velocity setpoint.
      Measure<Voltage> ffdOutput =
          Volts.of(feedforwardDrive.calculate(velocitySetpoint.in(MetersPerSecond)));
      Measure<Voltage> pidOutput =
          Volts.of(pidControllerVoltage.calculate(velocitySetpoint.in(MetersPerSecond)));

      // Averages voltage outputs.
      Measure<Voltage> voltage = ffdOutput.plus(pidOutput).divide(2);

      // Sets the voltages of the motors.
      differentialDrive.tankDrive(voltage.in(Volts), voltage.in(Volts));
    }).deadlineWith(updateRobotPose()).until(() -> pidControllerDrive.atGoal());
  }

  /**
   * Rotates the robot a certain orientation.
   * 
   * @param angle : angle to rotate the robot to(0 faces north, counterclockwise).
   * @return A command.
   */
  public Command rotateBy(Measure<Angle> angle) {
    // Sets the velocity PID controllers setpoint.
    pidControllerDrive.setGoal(getRobotOrientation().in(Radians) + angle.in(Radians));
    return run(() -> {
      // Calculates velocity setpoint.
      Measure<Velocity<Angle>> velocitySetpoint =
          RadiansPerSecond.of(pidControllerRotation.calculate(getPosition().in(Meters)));

      // Calculates voltage outputs based on velocity setpoint.
      Measure<Voltage> ffdOutput =
          Volts.of(feedforwardRotation.calculate(velocitySetpoint.in(RadiansPerSecond)));
      Measure<Voltage> pidOutput =
          Volts.of(pidControllerRotation.calculate(velocitySetpoint.in(RadiansPerSecond)));

      // Averages voltage outputs.
      Measure<Voltage> voltage = ffdOutput.plus(pidOutput).divide(2);

      // Sets the voltages of the motors.
      differentialDrive.tankDrive(voltage.in(Volts), voltage.in(Volts));
    }).deadlineWith(updateRobotPose()).until(() -> pidControllerDrive.atGoal());
  }

  @Override
  public void periodic() {
    motors.periodicMethod();
  }
}
