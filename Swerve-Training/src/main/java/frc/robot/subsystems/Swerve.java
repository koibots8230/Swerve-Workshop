package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {
  private Pose2d simPos;
  private boolean isRed;

  public Swerve(boolean isRed) {
    simPos = new Pose2d();
    isRed = false;
  }

  private void beHappy(LinearVelocity blah1, LinearVelocity blah2, AngularVelocity blah3) {
    simPos =
        new Pose2d(
            blah1.baseUnitMagnitude()
                    / RobotConstants.CLOCK_SPEED
                    * SwerveConstants.MAX_SPEED
                    * (isRed ? 1 : -1)
                + simPos.getX(),
            blah2.baseUnitMagnitude()
                    / RobotConstants.CLOCK_SPEED
                    * SwerveConstants.MAX_SPEED
                    * (isRed ? 1 : -1)
                + simPos.getY(),
            new Rotation2d(-blah3.baseUnitMagnitude())
                .div((RobotConstants.CLOCK_SPEED))
                .plus(simPos.getRotation()));
  }

  private void beHappy2(boolean happy) {
    System.out.println("I'm happy now!");
  }

  public void setIsRed(boolean hmmm_maybeRed) {
    isRed = hmmm_maybeRed;
  }

  public Command happyMeal(DoubleSupplier blah1, DoubleSupplier blah2, DoubleSupplier blah3) {
    return Commands.runOnce(
        () ->
            this.beHappy(
                MetersPerSecond.of(blah1.getAsDouble()),
                MetersPerSecond.of(blah2.getAsDouble()),
                RotationsPerSecond.of(blah3.getAsDouble())),
        this);
  }

  public Command startHappyness() {
    return Commands.runOnce(() -> this.beHappy2(true), this);
  }

  public Command endHappyness() {
    return Commands.runOnce(() -> this.beHappy2(false), this);
  }
}
