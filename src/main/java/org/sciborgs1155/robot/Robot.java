package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.robot.Ports.OI;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {

  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);


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
    // SmartDashboard.putData("PDH", new PowerDistribution());

    RobotController.setBrownoutVoltage(6.0);

    if (isReal()) {
      URCL.start();
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  /** Configures trigger -> command bindings */
  private void configureBindings() {}

  @Override
  public void close() {
    super.close();
  }
}
