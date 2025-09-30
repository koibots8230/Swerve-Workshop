package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {

  private boolean isBlue;
  @NotLogged private final CommandXboxController xboxController;
  private final Swerve swerve;

  public RobotContainer() {

    swerve = new Swerve();
    xboxController = new CommandXboxController(0);

    configureBindings();
    defualtCommands();
  }

  private void configureBindings() {}

  private void defualtCommands() {
    swerve.setDefaultCommand(
        swerve.driveCommand(
            xboxController::getLeftY, xboxController::getLeftX, xboxController::getRightX));
  }

  public void allianceColour() {
    isBlue = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    swerve.setIsBlue(isBlue);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
