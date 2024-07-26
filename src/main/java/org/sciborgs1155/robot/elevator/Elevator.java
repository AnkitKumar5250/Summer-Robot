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
 
     // instantiates motors
    private final TalonFX leader = new TalonFX(ELEVATOR_LEADER);
    private final TalonFX follower = new TalonFX(ELEVATOR_FOLLOWER);

     // instantiates PIDController
    private final PIDController pidController = new PIDController(kp, ki, kd);

     // instantiates encoder
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ELEVATOR_ENCODER_PIN);

    /**
     * Enumerator for describing certain elevation states.
     */
    enum Elevation {
        IDLE,
        GROUND,
        SMALL,
        LARGE,
    }

    /**
     * Instantiates elevator subsystem.
     */
    public Elevator() {
        // "A comand is... a command" - Ivan
        follower.setControl(new DifferentialFollower(leader.getDeviceID(), false));
        encoder.setDistancePerRotation(DISTANCE_PER_ROTATION);
    }

    /**
     * Moves elevator to a certain elevation state.
     * @param elevation : target elevation
     */
    private void move(Elevation elevation) {
        Measure<Distance> setpoint = Meters.of(0);

        switch (elevation) {
            case SMALL:
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
    }

    /**
     * Moves the elevator to the altitude for scoring in the small pole.
     * @return Command that moves the elevator to the altitude for scoring in the small pole.
     */
    public Command moveSmall() {
        return run(() -> move(Elevation.SMALL));
    }

    /**
     * Moves the elevator to the altitude for scoring in the large pole.
     * @return Command that moves the elevator to the altitude for scoring in the small pole.
     */
    public Command moveLarge() {
        return run(() -> move(Elevation.LARGE));
    }

    /**
     * Moves the elevator to the default position
     * @return Command that moves the elevator to the default position.
     */
    public Command moveGround() {
        return run(() -> move(Elevation.GROUND));
    }
}
