package org.sciborgs1155.robot.elevator;

import static org.sciborgs1155.robot.Ports.Elevator.*;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.ControlRequest.*;
import com.ctre.phoenix6.controls.DifferentialFollower;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final TalonFX leader = new TalonFX(ELEVATOR_LEADER);
    private final TalonFX follower = new TalonFX(ELEVATOR_FOLLOWER);

    private final PIDController pidController = new PIDController(kp, ki, kd);

    public Elevator() {
        follower.setControl(new DifferentialFollower(leader.getDeviceID(), false));
    }
    
    // I have to leave in abt 30 mins
    

    public void moveSmall() {
        
    }
}