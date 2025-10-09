package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Swerve extends SubsystemBase {

  public Swerve() {}

  private void beHappy(DoubleSupplier blah1, DoubleSupplier blah2, DoubleSupplier blah3) {
      System.out.println("I'm happy!");
      System.out.println(blah1 + " " + blah2 + " " + blah3);
  }

  private void beHappy2(boolean happy) {
    System.out.println("I'm happy now!");

  }

  public Command happyMeal(DoubleSupplier blah1, DoubleSupplier blah2, DoubleSupplier blah3) {
    return Commands.runOnce(() -> this.beHappy(blah1, blah2, blah3), this);
  
  }

  public Command startHappyness() {
    return Commands.runOnce(() -> this.beHappy2(true), this);

  }

  public Command endHappyness() {
    return Commands.runOnce(() -> this.beHappy2(false), this);

  }
}
