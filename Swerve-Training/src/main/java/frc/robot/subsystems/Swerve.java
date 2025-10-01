package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {

  private boolean isBlue;
  private Pose2d estimatedPose;
  private SwerveModuleState[] moduleStates;
  private ChassisSpeeds chassisSpeeds;

  public Swerve() {

    estimatedPose = new Pose2d();

    moduleStates = new SwerveModuleState[4];
  }

  public void setIsBlue(Boolean allianceColour) {
    isBlue = allianceColour;
  }

  private void fieldRelitiveDrive(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    estimatedPose =
        new Pose2d(
            ((x.baseUnitMagnitude() / 50 * 4) * (isBlue ? -1 : 1) + estimatedPose.getX()),
            (y.baseUnitMagnitude() / 50 * 4) * (isBlue ? -1 : 1) + estimatedPose.getY(),
            new Rotation2d(-omega.baseUnitMagnitude()).div(50).plus(estimatedPose.getRotation()));
  }

  public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () ->
            fieldRelitiveDrive(
                MetersPerSecond.of(x.getAsDouble()),
                MetersPerSecond.of(y.getAsDouble()),
                RotationsPerSecond.of(omega.getAsDouble())),
        this);
  }
}
