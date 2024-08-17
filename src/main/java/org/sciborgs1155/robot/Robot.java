package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.robot.Ports.Operator;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.tankdrive.TankDrive;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Logged;
import monologue.Monologue;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {

  // INPUT DEVICES
  @SuppressWarnings("unused")
  private final CommandXboxController operator = new CommandXboxController(Operator.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(Operator.DRIVER);

  // SUBSYSTEMS
  private final TankDrive drivetrain = new TankDrive();
  private final Elevator elevator = new Elevator();

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
    configureBindings();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    // Configure logging with DataLogManager, Monologue, FailureManagement, and URCL
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));

    SmartDashboard.putData(CommandScheduler.getInstance());

    // Log PDH
    SmartDashboard.putData("PDH", new PowerDistribution());

    RobotController.setBrownoutVoltage(6.0);

    if (isReal()) {
      URCL.start();
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }


  /** Configures trigger -> command bindings. */
  private void configureBindings() {
    // elevator moves up to the height appropriate for scoring in the large pole
    // when 'A' is pressed
    driver.a().onTrue(elevator.moveLarge());

    // elevator moves up to the height appropriate for scoring in the large pole
    // when 'B' is pressed
    driver.b().onTrue(elevator.moveSmall());

    // elevator moves up to the height appropriate for scoring in the large pole
    // when 'X' is pressed
    driver.x().onTrue(elevator.moveGround());
  }

  /**
   * Runs at the beggining of teleop.
   */
  @Override
  public void teleopInit() {
    // Starts driving based on driver input.
    drivetrain.setDefaultCommand(
        Commands.runOnce(() -> drivetrain.drive(driver.getLeftY(), driver.getRightY())));
  }

  /**
   * Runs at the beggining of autonomous.
   */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void close() {
    super.close();
  }
}
