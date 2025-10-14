package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.MetersPerSecond;
import static edu.wpi.first.units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.pos2d;
import edu.wpi.first.math.geometry.rotation2d;
import edu.wpi.first.math.kinimatics.chassisSpeeds;
import edu.wpi.first.math.kinimatics.SwerveModuleStates;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

private Pose2d simPos;

@Logged
public class Swerve extends SubsystemBase {

  public Swerve() {
  
    simPos = new Pose2d();
  }

  private void beHappy(LinearVelocity blah1, LinearVelocity blah2, AngularVelocity blah3) {
    simPos = new Pos2d(
      blah1.baseUnitMagnitude() / RobotConstants.CLOCK_SPEED * SwerveConstants.MAX_SPEED * (isBlue ? 1 : -1) + simPos.getX(),
      blah2.baseUnitMagnitude() / RobotConstants.CLOCK_SPEED * SwerveConstants.MAX_SPEED * (isBlue ? 1 : -1) + simPos.getY(),
      new Rotation2d(-blah3.baseUnitMagnitude()).div((RobotConstants.CLOCK_SPEED)).plus(simPos.getRotation())
        );
  }

  private void beHappy2(boolean happy) {
    System.out.println("I'm happy now!");

  }

  public Command happyMeal(DoubleSupplier blah1, DoubleSupplier blah2, DoubleSupplier blah3) {
    return Commands.runOnce(
      () -> this.beHappy(
        metersPerSecond.of(blah1.getAsDouble()), 
        metersPerSecond.of(blah2.getAsDouble()),
        rotationsPerSecond.of(blah3.getAsDouble())
      ), 
      this
    );
  
  }

  public Command startHappyness() {
    return Commands.runOnce(() -> this.beHappy2(true), this);

  }

  public Command endHappyness() {
    return Commands.runOnce(() -> this.beHappy2(false), this);

  }
}
