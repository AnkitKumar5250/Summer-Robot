package org.sciborgs1155.robot.wrist;

import org.sciborgs1155.robot.Ports.Drive;

import static org.sciborgs1155.robot.Ports.Wrist.*;

import static org.sciborgs1155.robot.wrist.WristConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{


    private final TalonFX wristMotor = new TalonFX(WRIST_MOTOR);
    private final PIDController pidController = new PIDController(kp, ki, kd);
    private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

    public Wrist(){
        setDefaultCommand(turnWrist(0));

        var wristConfigs = new TalonFXConfiguration();
        wristConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        wristConfigs.CurrentLimits.StatorCurrentLimit = 35;

        wristMotor.getConfigurator().apply(wristConfigs);
    }

    public Command turnWrist(double degrees){
        return Commands.run(() -> {{
            double currentAngle = wristMotor.getPosition().getValueAsDouble();
            wristMotor.setVoltage(pidController.calculate(currentAngle, degrees) + feedforward.calculate(degrees,0));
        }});
        
    }


}
