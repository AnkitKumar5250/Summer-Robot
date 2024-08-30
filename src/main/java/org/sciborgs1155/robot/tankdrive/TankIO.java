package org.sciborgs1155.robot.tankdrive;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

/** IO interface representing one side of a tank drivetrain. */
public interface TankIO {
  /**
   * Sets the voltage of both sides.
   * 
   * @param voltage : voltage.
   * @return Command.
   */
  public Command setVoltage(Measure<Voltage> voltage);

  /**
   * Sets the voltage of both sides.
   * 
   * @param voltage : volts.
   * @return Command.
   */
  public Command setVoltage(double volts);

  /**
   * Stops both sides.
   * 
   * @return Command.
   */
  public Command stop();

  /**
   * Inverts both sides.
   * 
   * @param invert : To invert, or to not invert.
   * @return Command.
   */
  public Command setInverted(boolean invert);

  /**
   * Inverts left side.
   * 
   * @param invert : To invert, or to not invert.
   * @return Command.
   */
  public Command setLeftInverted(boolean invert);

  /**
   * Sets the voltage of the left side.
   * 
   * @param voltage : voltage.
   * @return Command.
   */
  public Command setLeftVoltage(Measure<Voltage> voltage);

  /**
   * Sets the voltage of the left side.
   * 
   * @param voltage : volts.
   * @return Command.
   */
  public Command setLeftVoltage(double volts);

  /**
   * Stops the left side.
   * 
   * @return Command.
   */
  public Command stopLeft();

  /**
   * Returns the distance traveled(accounts for backwards travel) by the left side.
   * 
   * @return Distance.
   */
  public Measure<Distance> getLeftDistance();

  /**
   * Returns the velocity of the left side.
   * 
   * @return Velocity.
   */
  public Measure<Velocity<Distance>> getLeftVelocity();

  /**
   * Returns the voltage of the left side.
   * 
   * @return Voltage.
   */
  public Measure<Voltage> getLeftVoltage();

  /**
   * Inverts right side.
   * 
   * @param invert : To invert, or to not invert.
   * @return Command.
   */
  public Command setRightInverted(boolean invert);

  /**
   * Sets the voltage of the right side.
   * 
   * @param voltage : voltage.
   * @return Command.
   */
  public Command setRightVoltage(Measure<Voltage> voltage);

  /**
   * Sets the voltage of the right side.
   * 
   * @param voltage : volts.
   * @return Command.
   */
  public Command setRightVoltage(double volts);

  /**
   * Stops the right side.
   * 
   * @return Command.
   */
  public Command stopRight();

  /**
   * Returns the distance traveled(accounts for backwards travel) by the right side.
   * 
   * @return Distance.
   */
  public Measure<Distance> getRightDistance();

  /**
   * Returns the velocity of the right motor.
   * 
   * @return Velocity.
   */
  public Measure<Velocity<Distance>> getRightVelocity();

  /**
   * Returns the voltage of the right side.
   * 
   * @return Voltage.
   */
  public Measure<Voltage> getRightVoltage();

  /**
   * Default command for the motors.
   * 
   * @return Command.
   */
  public Command defaultCommand();

  /**
   * Called periodically.
   */
  public void periodicMethod();
}
