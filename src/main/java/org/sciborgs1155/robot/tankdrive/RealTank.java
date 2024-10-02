package org.sciborgs1155.robot.tankdrive;

import static org.sciborgs1155.robot.Ports.Drive.GYRO;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.MAXIMUM_VOLTAGE_MAGNITUDE;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.WHEEL_RADIUS;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.defaultGyroConfiguration;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.defaultTalonConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class RealTank implements TankIO, Subsystem {
    /** leader motor. */
    private TalonFX leftleader;

    /** follower motor. */
    private TalonFX leftfollower;

    /** leader motor. */
    private TalonFX rightleader;

    /** follower motor. */
    private TalonFX rightfollower;

    /** Gyroscope */
    private Pigeon2 gyro;

    @Override
    public Command setVoltages(Measure<Voltage> left, Measure<Voltage> right) {
        return runOnce(() -> setRightVoltage(right).alongWith(setLeftVoltage(left)));
    }

    @Override
    public Command setVoltage(Measure<Voltage> voltage) {
        return runOnce(() -> setRightVoltage(voltage).alongWith(setLeftVoltage(voltage)));
    }

    @Override
    public Command setPowers(double left, double right) {
        return runOnce(() -> setRightPower(left).alongWith(setLeftPower(right)));
    }

    @Override
    public Command setPower(double power) {
        return setPowers(power, power);
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
    public Command setLeftPower(double power) {
        return setLeftVoltage(
                Volts.of(MathUtil.clamp(power, -1, 1) * MAXIMUM_VOLTAGE_MAGNITUDE.in(Volts)));
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
    public Command setRightPower(double power) {
        return setRightVoltage(
                Volts.of(MathUtil.clamp(power, -1, 1) * MAXIMUM_VOLTAGE_MAGNITUDE.in(Volts)));
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

        // Instantiates gyroscope.
        gyro = new Pigeon2(GYRO);

        // Applies default gyro configuration.
        gyro.getConfigurator().apply(defaultGyroConfiguration);
    }

    @Override
    public Command defaultCommand() {
        return Commands.idle(this);
    }

    @Override
    public void periodicMethod() {
        return;
    }

    @Override
    public Rotation2d getGyroReading() {
        return gyro.getRotation2d();
    }

}
