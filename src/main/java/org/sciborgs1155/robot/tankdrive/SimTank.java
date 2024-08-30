package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.GEARING_RATIO;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.MOMENT_OF_INERTIA;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.ROBOT_MASS;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.STANDARD_DEVIATIONS;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.TRACK_WIDTH;
import static org.sciborgs1155.robot.tankdrive.TankDriveConstants.WHEEL_RADIUS;
import org.sciborgs1155.lib.GameTime;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimTank extends SubsystemBase implements TankIO {
    /** Simulated drivetrain. */
    private DifferentialDrivetrainSim drivetrainSim;

    /** Used for remembering voltage values. */
    private DifferentialDriveWheelVoltages storedVoltages;

    /** Used to track simulation time. */
    private Measure<Time> lastTime;
    private GameTime time;

    /** Used for graphical simulation. */
    private Field2d simulatedField = new Field2d();

    /**
     * Stores the set voltages of the sim motors(we can't access directly).
     * 
     * @param left : left voltage.
     * @param right : right voltage.
     */
    private void storeVoltages(Measure<Voltage> left, Measure<Voltage> right) {
        storedVoltages.left = left.in(Volts);
        storedVoltages.left = right.in(Volts);
    }

    /**
     * Stores the set voltages of the right sim motor(we can't access directly).
     * 
     * @param voltage : right voltage.
     */
    private void storeRightVoltage(Measure<Voltage> voltage) {
        storedVoltages.right = voltage.in(Volts);
    }

    /**
     * Stores the set voltages of the left sim motor(we can't access directly).
     * 
     * @param voltage : left voltage.
     */
    private void storeLeftVoltage(Measure<Voltage> voltage) {
        storedVoltages.left = voltage.in(Volts);
    }

    /**
     * Gets the amount of time elapsed since last call.
     * 
     * @return Time Difference since last call.
     */
    private Measure<Time> getDeltaTime() {
        Measure<Time> deltaTime = time.getTime().minus(lastTime);
        lastTime = time.getTime();
        return deltaTime;
    }

    @Override
    public Command setVoltage(Measure<Voltage> voltage) {
        return runOnce(() -> {
            drivetrainSim.setInputs(voltage.in(Volts), voltage.in(Volts));
            storeVoltages(voltage, voltage);
        });
    }

    @Override
    public Command setVoltage(double volts) {
        return setVoltage(Volts.of(volts));
    }

    @Override
    public Command setInverted(boolean invert) {
        return Commands.idle(this).withName("Invert command on sim drivetrain.");
    }

    @Override
    public Command stop() {
        return setVoltage(Volts.of(0));
    }

    @Override
    public Command setLeftVoltage(Measure<Voltage> voltage) {
        return runOnce(() -> {
            drivetrainSim.setInputs(voltage.in(Volts), storedVoltages.right);
            storeLeftVoltage(voltage);
        });
    }

    @Override
    public Command setLeftVoltage(double volts) {
        return setVoltage(Volts.of(volts));
    }

    @Override
    public Command stopLeft() {
        return setLeftVoltage(Volts.of(0));
    }

    /** Since the left side in a sim doesn't get inverted, this command is useless. */
    @Override
    public Command setLeftInverted(boolean invert) {
        return Commands.idle(this).withName("Invert left command on sim drivetrain.");
    }

    @Override
    public Command setRightVoltage(Measure<Voltage> voltage) {
        return runOnce(() -> {
            drivetrainSim.setInputs(storedVoltages.left, voltage.in(Volts));
            storeRightVoltage(voltage);
        });
    }

    @Override
    public Command setRightVoltage(double volts) {
        return setVoltage(Volts.of(volts));
    }

    @Override
    public Command stopRight() {
        return setRightVoltage(Volts.of(0));
    }

    /** Since the right side in a sim doesn't get inverted, this command is useless. */
    @Override
    public Command setRightInverted(boolean invert) {
        return Commands.idle(this).withName("Invert right command on sim drivetrain.");
    }

    @Override
    public Measure<Velocity<Distance>> getLeftVelocity() {
        return MetersPerSecond.of(drivetrainSim.getLeftVelocityMetersPerSecond());
    }

    @Override
    public Measure<Distance> getLeftDistance() {
        return Meters.of(drivetrainSim.getLeftPositionMeters());
    }

    @Override
    public Measure<Voltage> getLeftVoltage() {
        return Volts.of(storedVoltages.left);
    }


    @Override
    public Measure<Velocity<Distance>> getRightVelocity() {
        return MetersPerSecond.of(drivetrainSim.getRightVelocityMetersPerSecond());
    }

    @Override
    public Measure<Distance> getRightDistance() {
        return Meters.of(drivetrainSim.getRightPositionMeters());
    }

    @Override
    public Measure<Voltage> getRightVoltage() {
        return Volts.of(storedVoltages.right);
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
        return new SimTank(leftleaderMotorID, leftfollowerMotorID, rightleaderMotorID,
                rightfollowerMotorID);
    }

    /** Instantiates RealTank. */
    private SimTank(int leftleaderMotorID, int leftfollowerMotorID, int rightleaderMotorID,
            int rightfollowerMotorID) {
        drivetrainSim = new DifferentialDrivetrainSim(DCMotor.getKrakenX60(2), //
                GEARING_RATIO, // 7.29:1 gearing reduction.
                MOMENT_OF_INERTIA, // MOI of 7.5 kg m^2 (from CAD model).
                ROBOT_MASS.in(Kilograms), // The mass of the robot is 60 kg.
                WHEEL_RADIUS.in(Meters), // The robot uses 3" radius wheels.
                TRACK_WIDTH.in(Meters), // The track width is 0.7112 meters.
                STANDARD_DEVIATIONS // Standart deviations in measurement.
        );

        SmartDashboard.putData("Drivetrain Sim", simulatedField);
    }

    @Override
    public void periodicMethod() {
        // Updates simulation
        drivetrainSim.update(getDeltaTime().in(Seconds));
        simulatedField.setRobotPose(drivetrainSim.getPose());
    }

    @Override
    public Command defaultCommand() {
        return Commands.idle(this).withName("Default command on sim drivetrain.");
    }


}
