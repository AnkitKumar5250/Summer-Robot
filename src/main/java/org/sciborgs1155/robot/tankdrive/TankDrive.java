package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_RIGHT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_RIGHT_DRIVE;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.INITIAL_ROBOT_POSE;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.MAX_ACCELERATION;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.MINIMUM_DEGREE_THRESHOLD;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.MINIMUM_DISTANCE_THRESHOLD;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.TRACK_WIDTH;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Tank drivetrain Subsystem.
 */
public class TankDrive extends SubsystemBase {
  /** TankIO */
  private TankIO hardware;

  /** Commanded Voltages. */
  private DifferentialDriveWheelVoltages commandedVoltages;

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
      new DifferentialDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(0), 0, 0,
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
  private TankDrive(TankIO hardware, boolean isReal) {
    // Instantiates everything.
    this.hardware = hardware;
    this.commandedVoltages = new DifferentialDriveWheelVoltages(0, 0);

    // Sets the right to be inverted
    hardware.setRightInverted(true);

    // Adds data to the SmartDashboard.
    updateRobotPose();

    // Sets the tolerance of pid controllers
    pidControllerDrive.setTolerance(MINIMUM_DISTANCE_THRESHOLD.in(Meters));
    pidControllerRotation.setTolerance(MINIMUM_DEGREE_THRESHOLD.in(Degrees));

    // Sets default command.
    setDefaultCommand(run(() -> {

    }).withName("Drivetrain Default Command"));
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
    return Degrees.of(differentialDriveOdometry.getEstimatedPosition().getRotation().getDegrees());
  }

  /**
   * Returns distance traveled by the drivetrain(accounts for backwards travel).
   * 
   * @return Distance.
   */
  private Measure<Distance> getPosition() {
    return hardware.getLeftDistance().plus(hardware.getRightDistance()).divide(2);
  }

  private Command updateRobotPose() {
    return runOnce(() -> differentialDriveOdometry.update(hardware.getGyroReading(),
        hardware.getLeftDistance().in(Meters), hardware.getRightDistance().in(Meters)));
  }

  /**
   * Stops the drivetrain.
   * 
   * @return A command.
   */
  public Command stop() {
    return runOnce(() -> hardware.stop()).andThen(runOnce(() -> updateRobotPose()));
  }

  /**
   * Drives based on driver input(arcade). w
   * 
   * @param leftY : Y value of left joystick.
   * @param rightY : Y value of right joystick.
   * @return A command.
   */
  public Command driveTank(double leftY, double rightY) {
    return runOnce(() -> {
      commandedVoltages.left = MathUtil.clamp(leftY, -1, 1);
      commandedVoltages.right = MathUtil.clamp(rightY, -1, 1);
      updateRobotPose();
    }).withName("Tank drive(Left: " + hardware.getLeftDistance().in(Meters) + ";" + "Right: "
        + hardware.getRightDistance().in(Meters) + ")");
  }

  /**
   * Drives based on driver input(tank).
   * 
   * @param speed : Y value of joystick.
   * @param direction : X value of joystick.
   * @return A command.
   */
  public Command driveArcade(double speed, double direction) {
    return runOnce(() -> {
      commandedVoltages.left = MathUtil.clamp((speed + direction) / 2, -1, 1);
      commandedVoltages.right = MathUtil.clamp((speed - direction) / 2, -1, 1);
      updateRobotPose();
    }).withName("Arcade drive(Left: " + hardware.getLeftDistance().in(Meters) + ";" + "Right: "
        + hardware.getRightDistance().in(Meters) + ")");
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
    System.out.println("Drive command ran!");

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

      // Sets the voltages of the hardware.
      commandedVoltages.left = voltage.in(Volts);
      commandedVoltages.right = voltage.in(Volts);
    }).withName("Drive Distance(Error: " + pidControllerDrive.getPositionError() + ")")
        .andThen(periodicCommand()).until(() -> pidControllerDrive.atGoal());
  }

  /**
   * Rotates the robot a certain orientation.
   * 
   * @param angle : angle to rotate the robot to(0 faces north, counterclockwise).
   * @return A command.
   */
  public Command rotateBy(Measure<Angle> angle) {
    System.out.println("Rotate command ran!");
    // Sets the velocity PID controllers setpoint.
    pidControllerDrive.setGoal(getRobotOrientation().in(Radians) + angle.in(Radians));

    // Logs PID controller.
    SmartDashboard.putData(pidControllerDrive);
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

      // Sets the voltages of the hardware.
      commandedVoltages.left = -voltage.in(Volts);
      commandedVoltages.right = voltage.in(Volts);
    }).withName("Rotate Angle(Error: " + pidControllerRotation.getPositionError() + ")")
        .andThen(periodicCommand()).until(() -> pidControllerDrive.atGoal());
  }

  /**
   * Command to run periodically no matter the operating mode.
   * 
   * @return Command.
   */
  public Command periodicCommand() {
    return hardware.setVoltages(Volts.of(commandedVoltages.left), Volts.of(commandedVoltages.right))
        .andThen(runOnce(() -> {
          // Updates Smart Dashboard data.
          SmartDashboard.putString("Velocities",
              "L: " + hardware.getLeftVelocity().in(MetersPerSecond) + " R: "
                  + hardware.getRightVelocity().in(MetersPerSecond));

          SmartDashboard.putString("Commanded Voltages",
              "L: " + commandedVoltages.left + " R: " + commandedVoltages.right);

          SmartDashboard.putData(pidControllerDrive);
          SmartDashboard.putData(pidControllerRotation);
          SmartDashboard.putData(pidControllerVoltage);
          
          hardware.periodicMethod();
          updateRobotPose();
        }));
  }

  @Override
  public void periodic() {
    periodicCommand().schedule();
  }
}
