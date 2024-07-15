package org.sciborgs1155.robot.subsystem;

import javax.sound.sampled.Port;

import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Ports.Drive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SwerveDrive extends SubsystemBase {

    private static final MotorController frontLeft = new TalonFX(Drive.FRONT_LEFT_DRIVE);
    private static final MotorController rearLeft = new TalonFX(Drive.REAR_LEFT_DRIVE);
    private static final MotorController frontRight = new TalonFX(Drive.FRONT_RIGHT_DRIVE);
    private static final MotorController rearRight = new TalonFX(Drive.REAR_RIGHT_DRIVE);

    private static final PIDController pidController = new PIDController(1, 0, 1);

    public SwerveDrive() {
        
    }
    
}
