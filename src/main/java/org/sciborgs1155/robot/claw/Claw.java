package org.sciborgs1155.robot.claw;

import org.sciborgs1155.robot.Ports.Drive;

import static org.sciborgs1155.robot.Ports.Claw.CLAW_MOTOR;
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

// TODO: Rename Class
public class Claw extends SubsystemBase{


    private final TalonFX rollers = new TalonFX(CLAW_MOTOR);

    public Claw(){
        setDefaultCommand(stopRollers());

        var rollerConfigs = new TalonFXConfiguration();
        rollerConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        rollerConfigs.CurrentLimits.StatorCurrentLimit = 35;

        rollers.getConfigurator().apply(rollerConfigs);
    }

    public Command startRollers(){
        return Commands.runOnce(() -> rollers.set(1));
    }

    public Command stopRollers(){
        return Commands.runOnce(() -> rollers.set(0));
    }


}
