package org.sciborgs1155.robot.claw;

import org.sciborgs1155.robot.Ports.Drive;

import static org.sciborgs1155.robot.Ports.Claw.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{


    private final TalonFX motor = new TalonFX(CLAW_MOTOR);
    private final TalonFX rollers = new TalonFX(CLAW_ROLLERS);
    private final PIDController pidController = new PIDController(1, 0, 1);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(1,1);

    public Claw(){
    
    }

    public Command turnClaw(double degrees){

        return Commands.run(() -> {{
            double currentAngle = motor.getPosition().getValueAsDouble();
            motor.setVoltage(pidController.calculate(currentAngle, degrees));
        }});
        
    }

    public Command startRollers(){
        return Commands.runOnce(() -> rollers.set(1));
    }

    public Command stopRollers(){
        return Commands.runOnce(() -> rollers.set(0));
    }


}
