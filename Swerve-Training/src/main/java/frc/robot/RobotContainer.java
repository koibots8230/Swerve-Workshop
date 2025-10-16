package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {

  private final Swerve swerve;

  private final CommandXboxController xboxKid;

  public RobotContainer() {

    swerve = new Swerve();

    xboxKid = new CommandXboxController(RobotConstants.CONTROLLER_PORT);

    configureBindings();
    defualtCommands();
  }

  // private void blueGuy() {

  // }

  private void configureBindings() {
    Trigger buttonA = xboxKid.a();
    buttonA.onTrue(swerve.startHappyness());
    buttonA.onFalse(swerve.endHappyness());
  }

  private void defualtCommands() {
    swerve.setDefaultCommand(
        swerve.happyMeal(xboxKid::getLeftY, xboxKid::getLeftX, xboxKid::getRightX));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
