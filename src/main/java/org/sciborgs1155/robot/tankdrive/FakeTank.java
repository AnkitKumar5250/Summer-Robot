package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FakeTank implements TankIO, Subsystem {
    @Override
    public Command setVoltage(Measure<Voltage> voltage) {
        return Commands.idle(this).withName("Voltage command on fake drivetrain.");
    }

    @Override
    public Command setVoltage(double volts) {
        return Commands.idle(this).withName("Voltage command on fake drivetrain.");
    }

    @Override
    public Command setInverted(boolean invert) {
        return Commands.idle(this).withName("Invert command on fake drivetrain.");
    }

    @Override
    public Command stop() {
        return Commands.idle(this).withName("Stop command on fake drivetrain.");
    }

    @Override
    public Command setLeftInverted(boolean invert) {
        return Commands.idle(this).withName("Left invert command on fake drivetrain.");
    }

    @Override
    public Command setLeftVoltage(Measure<Voltage> voltage) {
        return Commands.idle(this).withName("Left voltage command on fake drivetrain.");
    }

    @Override
    public Command setLeftVoltage(double volts) {
        return Commands.idle(this).withName("Left voltage command on fake drivetrain.");
    }

    @Override
    public Command stopLeft() {
        return Commands.idle(this).withName("Left stop command on fake drivetrain.");
    }

    @Override
    public Measure<Velocity<Distance>> getLeftVelocity() {
        return MetersPerSecond.of(0);
    }

    @Override
    public Measure<Distance> getLeftDistance() {
        return Meters.of(0);
    }

    @Override
    public Measure<Voltage> getLeftVoltage() {
        return Volts.of(0);
    }

    @Override
    public Command setRightInverted(boolean invert) {
        return Commands.idle(this).withName("Right invert command on fake drivetrain.");
    }

    @Override
    public Command setRightVoltage(Measure<Voltage> voltage) {
        return Commands.idle(this).withName("Right voltage command on fake drivetrain.");
    }

    @Override
    public Command setRightVoltage(double volts) {
        return Commands.idle(this).withName("Right voltage command on fake drivetrain.");
    }

    @Override
    public Command stopRight() {
        return Commands.idle(this).withName("Right stop command on fake drivetrain.");
    }

    @Override
    public Measure<Velocity<Distance>> getRightVelocity() {
        return MetersPerSecond.of(0);
    }

    @Override
    public Measure<Distance> getRightDistance() {
        return Meters.of(0);
    }

    @Override
    public Measure<Voltage> getRightVoltage() {
        return Volts.of(0);
    }

    /**
     * Returns a new Instance of this TankIO class.
     * 
     * 
     * @return TankIO class.
     */
    public static TankIO create(int leftleaderMotorID, int leftfollowerMotorID,
            int rightleaderMotorID, int rightfollowerMotorID) {
        return new FakeTank(leftleaderMotorID, leftfollowerMotorID, rightleaderMotorID,
                rightfollowerMotorID);
    }

    private FakeTank(int leftleaderMotorID, int leftfollowerMotorID, int rightleaderMotorID,
            int rightfollowerMotorID) {}

    /**
     * Returns a new Instance of this TankIO class.
     * 
     * 
     * @return TankIO class.
     */
    public static TankIO create() {
        return new FakeTank();
    }

    private FakeTank() {}

    @Override
    public Command defaultCommand() {
        return Commands.idle(this);
    }

    @Override
    public void periodicMethod() {
        return;
    }
}
