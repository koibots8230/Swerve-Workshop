package frc.robot;

import java.io.Console;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  
  private final Swerve swerve;

  private final CommandXboxController xboxKid;

  public RobotContainer() {

    swerve = new Swerve();

    xboxKid = new CommandXboxController(0);

    configureBindings();
    defualtCommands();
  }

  

  private void configureBindings() {
    swerve.setDefaultCommand(
      swerve.happyMeal(
        Double.valueOf(xboxKid::getLeftY), 
        Double.valueOf(xboxKid::getLeftX), 
        Double.valueOf(xboxKid::getRight)
      )
    );
  
    Trigger buttonA = xboxKid.a();
    buttonA.onTrue(System.out.println("A button pressed"))
    buttonA.onFalse(System.out.println("A button released"));
  
  }


  private void defualtCommands() {}


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
