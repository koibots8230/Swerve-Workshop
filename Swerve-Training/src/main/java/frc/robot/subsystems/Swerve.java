package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Swerve extends SubsystemBase {

  public Swerve() {}

  private void beHappy(double blah1, double blah2, double blah3) {
    System.out.println("I'm happy!");
    System.out.println(blah1 + " " + blah2 + " " + blah3);
  }

  public Command happyMeal(double blah1, double blah2, double blah3) {
    return Commands.runOnce(() -> this.beHappy(blah1, blah2, blah3), this);
  
  }
}
