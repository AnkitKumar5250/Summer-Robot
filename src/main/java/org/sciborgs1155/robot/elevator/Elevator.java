package org.sciborgs1155.robot.elevator;

import static org.sciborgs1155.robot.Ports.Elevator.*;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DifferentialFollower;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final TalonFX leader = new TalonFX(ELEVATOR_LEADER);
    private final TalonFX follower = new TalonFX(ELEVATOR_FOLLOWER);

    private final PIDController pidController = new PIDController(kp, ki, kd);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ELEVATOR_ENCODER_PIN);

    private state status = state.IDLE;

    enum state {
        IDLE,
        MOVING_SMALL,
        MOVING_MEDIUM,
        MOVING_LARGE,
    }

    public Elevator() {
        follower.setControl(new DifferentialFollower(leader.getDeviceID(), false));
        encoder.setDistancePerRotation(DISTANCE_PER_ROTATION);
        encoder.reset();
    }

    public void moveSmall() {
        if (status != state.IDLE) {
            return;
        }
        status = state.MOVING_SMALL;
        encoder.reset();
        while (encoder.getDistance() - SMALL_POLE.baseUnitMagnitude() < TOLERANCE) {
            pidController.calculate(encoder.getDistance(), SMALL_POLE.baseUnitMagnitude());
        }
        encoder.reset();
        status = state.IDLE;
    }
    public void moveMedium() {
        if (status != state.IDLE) {
            return;
        }
        status = state.MOVING_MEDIUM;
        encoder.reset();
        while (encoder.getDistance() - MEDUIM_POLE.baseUnitMagnitude() < TOLERANCE) {
            pidController.calculate(encoder.getDistance(), MEDUIM_POLE.baseUnitMagnitude());
        }
        encoder.reset();
        status = state.IDLE;
    }
    public void moveLarge() {
        if (status != state.IDLE) {
            return;
        }
        status = state.MOVING_LARGE;
        encoder.reset();
        while (encoder.getDistance() - LARGE_POLE.baseUnitMagnitude() < TOLERANCE) {
            pidController.calculate(encoder.getDistance(), LARGE_POLE.baseUnitMagnitude());
        }
        encoder.reset();
        status = state.IDLE;
    }

    public state getStatus() {
        return status;
    }
}