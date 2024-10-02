package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.robot.Ports.Operator;
import org.sciborgs1155.robot.tankdrive.TankDrive;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {

  /** Controller for everything besides drivetrain. */
  @SuppressWarnings("unused")
  private final CommandXboxController operator = new CommandXboxController(Operator.OPERATOR);

  /** Controller for drivetrain. */
  private final CommandXboxController driver = new CommandXboxController(Operator.DRIVER);

  /** Allows for keyboard control. */
  private final Joystick keyboard = new Joystick(Operator.KEYBOARD_JOYSTICK);

  /** Drivetrain subsystem. */
  private final TankDrive drivetrain = TankDrive.sim();

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    // Configure logging with DataLogManager, Monologue, FailureManagement, and URCL
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));

    // Adds sceduled commands to dashboard.
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Log PDH
    SmartDashboard.putData("PDH", new PowerDistribution());

    // Sets the brownout voltage of the robot.
    RobotController.setBrownoutVoltage(6.0);

    if (isReal()) {
      URCL.start();
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }


  /** Configures trigger -> command bindings. */
  private void configureBindings() {

  }

  @Override
  public void teleopInit() {
    // Cancels all running commands at the beggining of teleop.
    CommandScheduler.getInstance().cancelAll();

    // Configures button bindings.
    configureBindings();
  }

  @Override
  public void teleopPeriodic() {
    if (Robot.isReal()) {
      drivetrain.driveTank(driver.getLeftY(), driver.getRightY()).schedule();
    }
    if (!Robot.isReal()) {
      drivetrain.driveArcade(keyboard.getRawAxis(1), keyboard.getRawAxis(0)).schedule();
    }
  }

  @Override
  public void autonomousInit() {
    // Cancels all running commands at the beggining of autonomous.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void autonomousPeriodic() {
    if (Robot.isReal()) {
      drivetrain.driveTank(driver.getLeftY(), driver.getRightY()).schedule();
    }
    if (!Robot.isReal()) {
      if (keyboard.getRawButtonPressed(1)) {
        drivetrain.drive(Meters.of(1)).schedule();
      }
      if (keyboard.getRawButtonPressed(2)) {
        drivetrain.rotateBy(Degrees.of(90)).schedule();
      }
    }
  }

  @Override
  public void close() {
    super.close();
  }
}
