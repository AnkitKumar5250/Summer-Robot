package org.sciborgs1155.robot.drive;

import org.sciborgs1155.robot.Ports.Drive;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase {

    // initializes motors
    private final MotorController frontLeft = new TalonFX(Drive.FRONT_LEFT_DRIVE);
    private final MotorController rearLeft = new TalonFX(Drive.REAR_LEFT_DRIVE);
    private final MotorController frontRight = new TalonFX(Drive.FRONT_RIGHT_DRIVE);
    private final MotorController rearRight = new TalonFX(Drive.REAR_RIGHT_DRIVE);

    private final DutyCycleEncoder frontLeftEncoder = new DutyCycleEncoder(Drive.FRONT_LEFT_ENCODER_PIN);
    private final DutyCycleEncoder rearLeftEncoder = new DutyCycleEncoder(Drive.REAR_LEFT_ENCODER_PIN);
    private final DutyCycleEncoder frontRightEncoder = new DutyCycleEncoder(Drive.FRONT_RIGHT_ENCODER_PIN);
    private final DutyCycleEncoder rearRightEncoder = new DutyCycleEncoder(Drive.REAR_RIGHT_ENCODER_PIN);

    // initializes PID controllers
    private final PIDController pidControllerDrive = new PIDController(1, 0, 1);
    private final PIDController pidControllerRotation = new PIDController(1, 0, 1);

    public TankDrive() {
        
    }

    private void updateDirection(double degrees) {
        double encoderValue = frontLeftEncoder.get() + rearLeftEncoder.get() + frontRightEncoder.get() + rearRightEncoder.get()/4;
        double voltage = pidControllerRotation.calculate(encoderValue/2, degrees * TankDriveConstants.DISTANCE_PER_DEGREE);
        
        frontLeft.setVoltage(-voltage);
        rearLeft.setVoltage(-voltage);
        frontRight.setVoltage(voltage);
        rearRight.setVoltage(voltage);
    }

    public Command setDirection(double degrees) {
        frontLeftEncoder.reset();
        rearLeftEncoder.reset();
        frontRightEncoder.reset();
        rearRightEncoder.reset();

        return run(() -> updateDirection(degrees));
    }


    
}
