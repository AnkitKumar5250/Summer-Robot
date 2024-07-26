package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.TankDriveConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TankDrive extends SubsystemBase {

    // instantiates motors
    private final TalonFX frontLeft = new TalonFX(FRONT_LEFT_DRIVE);
    @SuppressWarnings("unused")
    private final TalonFX rearLeft = new TalonFX(REAR_LEFT_DRIVE);
    private final TalonFX frontRight = new TalonFX(FRONT_RIGHT_DRIVE);
    @SuppressWarnings("unused")
    private final TalonFX rearRight = new TalonFX(REAR_RIGHT_DRIVE);

    // instantiates encoders
    private final DutyCycleEncoder leftEncoder = new DutyCycleEncoder(FRONT_LEFT_ENCODER_PIN);
    private final DutyCycleEncoder rightEncoder = new DutyCycleEncoder(FRONT_RIGHT_ENCODER_PIN);

    // instantiates PID controllers
    private final PIDController pidControllerDrive = new PIDController(1, 0, 1);
    private final PIDController pidControllerRotation = new PIDController(1, 0, 1);
    
    // instantiates differential drive
    private final DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft::setVoltage,frontRight::setVoltage);
  
    /**
     * Instantiates drivetrain.
     */
    public TankDrive() {
        TalonFXConfiguration defaultConfiguration = new TalonFXConfiguration();

        frontLeft.getConfigurator().apply(defaultConfiguration);
        frontRight.getConfigurator().apply(defaultConfiguration);

        frontLeft.setControl(new Follower(REAR_LEFT_DRIVE, false));
        frontRight.setControl(new Follower(REAR_RIGHT_DRIVE, false));

        frontRight.setInverted(true);

        leftEncoder.setDistancePerRotation(DISTANCE_PER_ROTATION.in(Meters));
        rightEncoder.setDistancePerRotation(DISTANCE_PER_ROTATION.in(Meters));
    }

    /**
     * Updates motor voltage based on PID.
     * @param degrees : the angle the robot is moving towards.
     */
    private void updateDirection(Measure<Angle> degrees) {
        double encoderValue = leftEncoder.get() + rightEncoder.get()/2;
        double distance = degrees.in(Degrees) * TURNING_RADIUS.in(Meters) * Math.PI * 2 / 360;
        double voltage = pidControllerDrive.calculate(encoderValue/2, distance);
        
        frontLeft.setVoltage(-voltage);
        frontRight.setVoltage(voltage);
    }

    /**
     * Updates motor voltage based on PID.
     * @param distance : the distance the robot is moving towards.
     */
    private void updateVelocity(Measure<Distance> distance) {
        double encoderValue = leftEncoder.get() + rightEncoder.get()/2;
        double voltage = pidControllerRotation.calculate(encoderValue/2, distance.in(Meters));
        
        frontLeft.setVoltage(voltage);
        frontRight.setVoltage(voltage);
    }

    /**
     * Updates motor voltage based on drive input.
     * @param controller : XboxController.
     */
    public void updateVelocity(double leftY, double rightY) {
        differentialDrive.tankDrive(leftY, rightY);
    }

    /**
     * Command that drives a certain distance.
     * @param distance : distance to move the robot.
     * @return Command that drives a certain distance.
     */
    public Command drive(Measure<Distance> distance) {
        leftEncoder.reset();
        rightEncoder.reset();

        return run(() -> updateVelocity(distance));
    }

    /**
     * Command that drives based on driver input
     * @param controller : XboxController
     * @return Command that drives based on driver input
     */
    public Command drive(double leftY,double rightY) {
        return run(() -> updateVelocity(leftY,rightY));
    }

    /**
     * Command that rotates the robot a certain amount of degrees.
     * @param degrees : amount of degrees to move the robot.
     * @return Command that rotates a certain amount of degrees.
     */
    public Command setDirection(Measure<Angle> degrees) {
        leftEncoder.reset();
        rightEncoder.reset();

        return run(() -> updateDirection(degrees));
    }    
}
