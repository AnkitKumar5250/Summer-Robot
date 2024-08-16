package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.Constants.Field.GROUND;
import static org.sciborgs1155.robot.Constants.Field.LARGE_POLE;
import static org.sciborgs1155.robot.Constants.Field.MEDUIM_POLE;
import static org.sciborgs1155.robot.Ports.Elevator.ELEVATOR_ENCODER_PIN;
import static org.sciborgs1155.robot.Ports.Elevator.ELEVATOR_FOLLOWER;
import static org.sciborgs1155.robot.Ports.Elevator.ELEVATOR_LEADER;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.DISTANCE_PER_ROTATION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MINIMUM_VELOCITY_THRESHOLD;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.ka;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kd;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kg;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.ki;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kp;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.ks;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kv;

import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Elevation;


/**
 * Elevator subsystem.
 */
public class Elevator extends SubsystemBase {

  // Instantiates motors
  private final TalonFX leader = new TalonFX(ELEVATOR_LEADER);
  private final TalonFX follower = new TalonFX(ELEVATOR_FOLLOWER);

  // Instantiates PID Controller
  private final PIDController pidController = new PIDController(kp, ki, kd);
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(ks, kg, kv, ka);

  // Instantiates encoder
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(ELEVATOR_ENCODER_PIN);

  /**
   * Instantiates elevator subsystem.
   */
  public Elevator() {
    // Sets follower and leader
    follower.setControl(new DifferentialFollower(leader.getDeviceID(), false));

    // Sets Distance per rotation of the motor
    encoder.setDistancePerRotation(DISTANCE_PER_ROTATION.in(Meters));

    // Sets the tolerance of pid controller
    pidController.setTolerance(MINIMUM_VELOCITY_THRESHOLD.in(MetersPerSecond));

    // Sets the default command to move to the ground state
    setDefaultCommand(move(Elevation.GROUND));
  }

  /**
   * Moves elevator to a certain elevation state.

   * @param elevation : target elevation
   */
  private Command move(Elevation elevation) {
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

    pidController.setSetpoint(setpoint.in(Meters));

    return run(() -> {
      double voltage = (pidController.calculate(encoder.getDistance())
          + feedforward.calculate(pidController.getSetpoint())) / 2;
      leader.setVoltage(voltage);
    });
  }

  /**
   * Moves the elevator to the altitude for scoring in the small pole.

   * @return A command.
   */
  public Command moveSmall() {
    return move(Elevation.SMALL).until(() -> pidController.atSetpoint());
  }

  /**
   * Moves the elevator to the altitude for scoring in the large pole.

   * @return A command.
   */
  public Command moveLarge() {
    return move(Elevation.LARGE).until(() -> pidController.atSetpoint());
  }

  /**
   * Moves the elevator to the default position.

   * @return A command.
   */
  public Command moveGround() {
    return move(Elevation.GROUND).until(() -> pidController.atSetpoint());
  }
}
