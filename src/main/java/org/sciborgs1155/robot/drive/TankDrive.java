package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.TankDriveConstants.*;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase {

    // instantiates motors
    private final TalonFX frontLeft = new TalonFX(FRONT_LEFT_DRIVE);
    private final TalonFX rearLeft = new TalonFX(REAR_LEFT_DRIVE);
    private final TalonFX frontRight = new TalonFX(FRONT_RIGHT_DRIVE);
    private final TalonFX rearRight = new TalonFX(REAR_RIGHT_DRIVE);

    // instantiates gyro
    private final Pigeon2 gyro = new Pigeon2(GYRO);

    // instantiates encoders
    private final DutyCycleEncoder leftEncoder = new DutyCycleEncoder(FRONT_LEFT_ENCODER_PIN);
    private final DutyCycleEncoder rightEncoder = new DutyCycleEncoder(FRONT_RIGHT_ENCODER_PIN);

    // instantiates PID controllers
    private final PIDController pidControllerTranslation = new PIDController(1, 0, 1);
    private final PIDController pidControllerRotation = new PIDController(1, 0, 1);

    // instantiates differential drive
    private final DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft::setVoltage,
            frontRight::setVoltage);

    // instantiates Odometry classes
    private final DifferentialDriveWheelPositions differentialDriveWheelPositions = new DifferentialDriveWheelPositions(
            Meters.of(0), Meters.of(0));
    private final DifferentialDriveOdometry differentialDriveOdometry = new DifferentialDriveOdometry(
            gyro.getRotation2d(), Meters.of(0), Meters.of(0), INITIAL_ROBOT_POSE);

    /**
     * Instantiates drivetrain.
     */
    public TankDrive() {
        // Creates a new instance of the default talon configuration.
        TalonFXConfiguration defaultConfiguration = new TalonFXConfiguration();

        // Creates a new instance of the default gyro configuration.
        Pigeon2Configuration defaultGyroConfiguration = new Pigeon2Configuration();

        // Applies default talon configuration to motors.
        frontLeft.getConfigurator().apply(defaultConfiguration);
        frontRight.getConfigurator().apply(defaultConfiguration);
        rearLeft.getConfigurator().apply(defaultConfiguration);
        rearRight.getConfigurator().apply(defaultConfiguration);

        defaultConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        frontLeft.getConfigurator().apply(defaultConfiguration);
        frontRight.getConfigurator().apply(defaultConfiguration);
        rearLeft.getConfigurator().apply(defaultConfiguration);
        rearRight.getConfigurator().apply(defaultConfiguration);

        // Applies default gyro configuration.
        gyro.getConfigurator().apply(defaultGyroConfiguration);

        // Sets leaders and followers
        frontLeft.setControl(new Follower(rearLeft.getDeviceID(), false));
        frontRight.setControl(new Follower(rearRight.getDeviceID(), false));

        // Sets the right to be inverted
        frontRight.setInverted(true);

        // Sets the distance per one rotation of the motors
        leftEncoder.setDistancePerRotation(DISTANCE_PER_ROTATION.in(Meters));
        rightEncoder.setDistancePerRotation(DISTANCE_PER_ROTATION.in(Meters));

        // Sets the tolerance of pid controllers
        pidControllerTranslation.setTolerance(MINIMUM_DISTANCE_THRESHOLD.in(Meters));
        pidControllerRotation.setTolerance(MINIMUM_DEGREE_THRESHOLD.in(Degrees));

        // Sets the default command to stop
        setDefaultCommand(run(() -> {
            frontRight.setVoltage(pidControllerTranslation.calculate(frontRight.getVelocity().getValueAsDouble(), 0));
            frontLeft.setVoltage(pidControllerTranslation.calculate(frontLeft.getVelocity().getValueAsDouble(), 0));
        }).until(() -> pidControllerTranslation.atSetpoint()));
    }

    /**
     * Resets the encoders and updates the robot position on the field.
     */
    public void updateWheelPositions() {
        differentialDriveWheelPositions.leftMeters += leftEncoder.getDistance();
        differentialDriveWheelPositions.rightMeters += rightEncoder.getDistance();

        leftEncoder.reset();
        rightEncoder.reset();
    }

    /**
     * Gets the robots current position on the field.
     * 
     * @return the position of the robot.
     */
    public Pose2d getRobotPose() {
        return differentialDriveOdometry.getPoseMeters();
    }

    /**
     * Stops the drivetrain.
     * 
     * @return A command.
     */
    public Command stop() {
        return run(() -> {
            frontRight.setVoltage(pidControllerTranslation.calculate(frontRight.getVelocity().getValueAsDouble(), 0));
            frontLeft.setVoltage(pidControllerTranslation.calculate(frontLeft.getVelocity().getValueAsDouble(), 0));
        }).until(() -> pidControllerTranslation.atSetpoint());
    }

    /**
     * Drives based on driver input.
     * 
     * @param controller : Command Xbox Controller
     * @return A command.
     */
    public Command drive(double leftY, double rightY) {
        updateWheelPositions();
        return run(() -> differentialDrive.tankDrive(leftY, rightY)).alongWith(runOnce(() -> {
            differentialDriveOdometry.update(gyro.getRotation2d(), differentialDriveWheelPositions);
        }));
    }

    /**
     * Drives the robot a certain distance.
     * 
     * @param distance : distance to move the robot.
     * @return A command.
     */
    public Command drive(Measure<Distance> distance) {
        updateWheelPositions();
        pidControllerTranslation.setSetpoint(distance.in(Meters));

        return run(() -> {
            double encoderValue = (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
            double voltage = pidControllerTranslation.calculate(encoderValue);

            frontLeft.setVoltage(voltage);
            frontRight.setVoltage(voltage);
        }).alongWith(runOnce(() -> {
            differentialDriveOdometry.update(gyro.getRotation2d(), differentialDriveWheelPositions);
        })).until(() -> pidControllerTranslation.atSetpoint());
    }

    /**
     * Rotates the robot a certain amount of degrees.
     * 
     * @param degrees : amount of degrees to rotate the robot.
     * @return A command.
     */
    public Command rotateBy(Measure<Angle> degrees) {
        updateWheelPositions();

        double distance = degrees.in(Degrees) * TURNING_RADIUS.in(Meters) * Math.PI * 2 / 360;
        pidControllerRotation.setSetpoint(distance);

        return run(() -> {
            double encoderValue = (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
            double voltage = pidControllerRotation.calculate(encoderValue);

            frontLeft.setVoltage(-voltage);
            frontRight.setVoltage(voltage);
        }).alongWith(runOnce(() -> {
            differentialDriveOdometry.update(gyro.getRotation2d(), differentialDriveWheelPositions);
        })).until(() -> pidControllerRotation.atSetpoint());
    }
}
