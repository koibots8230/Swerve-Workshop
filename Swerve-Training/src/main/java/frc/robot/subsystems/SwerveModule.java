package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveModule {

  private Angle turnSetpointAngle;
  private LinearVelocity driveSetpointVelocity;

  public SwerveModule() {}

  public void setState(SwerveModuleState state) {
    driveSetpointVelocity = MetersPerSecond.of(state.speedMetersPerSecond);
    turnSetpointAngle = Radians.of(state.angle.getRadians());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveSetpointVelocity, new Rotation2d(turnSetpointAngle));
  }
}
