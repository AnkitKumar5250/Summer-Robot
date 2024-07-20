package org.sciborgs1155.robot.elevator;

import static org.sciborgs1155.robot.Ports.Elevator.*;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DifferentialFollower;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance; 
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final TalonFX leader = new TalonFX(ELEVATOR_LEADER);
    private final TalonFX follower = new TalonFX(ELEVATOR_FOLLOWER);

    private final PIDController pidController = new PIDController(kp, ki, kd);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ELEVATOR_ENCODER_PIN);
    
    private ElevatorState status = ElevatorState.IDLE;
 
    enum ElevatorState {
        IDLE,
        MOVING_GROUND,
        MOVING_MEDIUM,
        MOVING_LARGE,
    }

    enum Elevation {
        IDLE,
        GROUND,
        MEDIUM,
        LARGE,
    }

    public Elevator() {
        // "A comand is... a command" - Ivan
        follower.setControl(new DifferentialFollower(leader.getDeviceID(), false));
        encoder.setDistancePerRotation(DISTANCE_PER_ROTATION);
    }

    public void move(Elevation elevation) {
        if (status != ElevatorState.IDLE) {
            return;
        }

        Measure<Distance> setpoint = Meters.of(0);

        switch (elevation) {
            case MEDIUM:
                setpoint = MEDUIM_POLE;
                break;
            case GROUND:
                setpoint = GROUND;
                break;
            case LARGE:
                setpoint = LARGE_POLE;
                break;
            default: 
                break;
        }

        while (Math.abs(encoder.getDistance() - setpoint.in(Meters)) < TOLERANCE) { 
            leader.setVoltage(pidController.calculate(encoder.getDistance(), setpoint.in(Meters)));
        }

        status = ElevatorState.IDLE;
    }

    public ElevatorState getStatus() {
        return status;
    }

    public Command moveMedium() {
        return run(() -> move(Elevation.MEDIUM));
    }
    public Command moveLarge() {
        return run(() -> move(Elevation.LARGE));
    }
    public Command moveGround() {
        return run(() -> move(Elevation.GROUND));
    }
}

