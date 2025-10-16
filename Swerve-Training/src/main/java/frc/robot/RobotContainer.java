package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  public static boolean isRed;

  public RobotContainer() {

    swerve = new Swerve(isRed);

    xboxKid = new CommandXboxController(RobotConstants.CONTROLLER_PORT);

    isRed = false;

    configureBindings();
    defualtCommands();
  }

  public void iWonder_IsItRed() {
    isRed = (DriverStation.getAlliance().get() == Alliance.Red);
    swerve.setIsRed(isRed);
  }

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
