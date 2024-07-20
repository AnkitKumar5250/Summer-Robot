package org.sciborgs1155.robot.claw;

import org.sciborgs1155.robot.Ports.Drive;

import static org.sciborgs1155.robot.Ports.Claw.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Claw {

    private final TalonFX motor = new TalonFX(CLAW_MOTOR);
    private final TalonFX rollers = new TalonFX(CLAW_ROLLERS);
    private final PIDController pidController = new PIDController(1, 0, 1);

    public Claw(){
    
    }

    public void turnClaw(double degrees){

        double currentAngle = motor.getPosition().getValueAsDouble();

        while( currentAngle != degrees ){
            currentAngle = motor.getPosition().getValueAsDouble();
            motor.setVoltage(pidController.calculate(currentAngle, degrees));
        }
        
    }

    public void startRollers(){
        rollers.set(1);
    }

    public void stopRollers(){
        rollers.set(0);
    }


}
