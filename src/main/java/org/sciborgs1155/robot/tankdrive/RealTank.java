package org.sciborgs1155.robot.tankdrive;

import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.WHEEL_RADIUS;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.defaultTalonConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class RealTank extends SubsystemBase implements TankIO {
    /** leader motor. */
    private TalonFX leftleader;

    /** follower motor. */
    private TalonFX leftfollower;

    /** leader motor. */
    private TalonFX rightleader;

    /** follower motor. */
    private TalonFX rightfollower;

    @Override
    public Command setVoltage(Measure<Voltage> voltage) {
        return runOnce(() -> setRightVoltage(voltage).alongWith(setLeftVoltage(voltage)));
    }

    @Override
    public Command setVoltage(double volts) {
        // Converts volts to arbitrary units and runs 'setVoltage' command.
        return setVoltage(Volts.of(volts));
    }

    @Override
    public Command setInverted(boolean invert) {
        return runOnce(() -> setLeftInverted(invert).alongWith(setRightInverted(invert)));
    }

    @Override
    public Command stop() {
        return runOnce(() -> stopRight().alongWith(stopLeft()));
    }

    @Override
    public Command setLeftVoltage(Measure<Voltage> voltage) {
        return runOnce(() -> leftleader.setVoltage(voltage.in(Volts)));
    }

    @Override
    public Command setLeftVoltage(double volts) {
        return setLeftVoltage(Volts.of(volts));
    }

    @Override
    public Command setLeftInverted(boolean invert) {
        return runOnce(() -> leftleader.setInverted(invert));
    }

    @Override
    public Command stopLeft() {
        return runOnce(() -> leftleader.stopMotor());
    }

    @Override
    public Command setRightVoltage(Measure<Voltage> voltage) {
        return runOnce(() -> rightleader.setVoltage(voltage.in(Volts)));
    }

    @Override
    public Command setRightVoltage(double volts) {
        return setRightVoltage(Volts.of(volts));
    }

    @Override
    public Command setRightInverted(boolean invert) {
        return runOnce(() -> rightleader.setInverted(invert));
    }

    @Override
    public Command stopRight() {
        return runOnce(() -> rightleader.stopMotor());
    }

    @Override
    public Measure<Velocity<Distance>> getLeftVelocity() {
        // Converts rotations per second into meters per second(and finds the average velocity).
        return MetersPerSecond.of(WHEEL_RADIUS.times(Math.PI * 2)
                .times(leftleader.getVelocity().getValueAsDouble()).in(Meters));
    }


    @Override
    public Measure<Distance> getLeftDistance() {
        // Converts rotations into meters.
        return WHEEL_RADIUS.times(Math.PI * 2).times(leftleader.getPosition().getValueAsDouble());
    }

    @Override
    public Measure<Voltage> getLeftVoltage() {
        return Volts.of(leftleader.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public Measure<Velocity<Distance>> getRightVelocity() {
        // Converts rotations per second into meters per second(and finds the average velocity).
        return MetersPerSecond.of(WHEEL_RADIUS.times(Math.PI * 2)
                .times(rightleader.getVelocity().getValueAsDouble()).in(Meters));
    }


    @Override
    public Measure<Distance> getRightDistance() {
        // Converts rotations into meters.
        return WHEEL_RADIUS.times(Math.PI * 2).times(rightleader.getPosition().getValueAsDouble());
    }

    @Override
    public Measure<Voltage> getRightVoltage() {
        return Volts.of(rightleader.getMotorVoltage().getValueAsDouble());
    }

    /**
     * Returns a new Instance of this TankIO class.
     * 
     * @param leftleaderMotorID : ID of the left leader motor.
     * @param leftfollowerMotorID : ID of the left follower motor.
     * @param rightleaderMotorID : ID of the right leader motor.
     * @param rightfollowerMotorID : ID of the right follower motor.
     * @return TankIO class.
     */
    public static TankIO create(int leftleaderMotorID, int leftfollowerMotorID,
            int rightleaderMotorID, int rightfollowerMotorID) {
        return new RealTank(leftleaderMotorID, leftfollowerMotorID, rightleaderMotorID,
                rightfollowerMotorID);
    }

    /** Instantiates RealTank. */
    private RealTank(int leftleaderMotorID, int leftfollowerMotorID, int rightleaderMotorID,
            int rightfollowerMotorID) {
        // Instantiates motors.
        leftleader = new TalonFX(leftleaderMotorID);
        leftfollower = new TalonFX(leftfollowerMotorID);
        rightleader = new TalonFX(rightleaderMotorID);
        rightfollower = new TalonFX(rightfollowerMotorID);

        // Applies default talon configuration to motors.
        leftleader.getConfigurator().apply(defaultTalonConfiguration);
        leftfollower.getConfigurator().apply(defaultTalonConfiguration);
        rightleader.getConfigurator().apply(defaultTalonConfiguration);
        rightfollower.getConfigurator().apply(defaultTalonConfiguration);

        // Makes the follower motor follow the leader motor.
        leftleader.setControl(new Follower(leftfollower.getDeviceID(), false));
        rightleader.setControl(new Follower(rightfollower.getDeviceID(), false));
    }

    @Override
    public Command defaultCommand() {
        return Commands.idle(this);
    }

    @Override
    public void periodicMethod() {
        return;
    }

}
