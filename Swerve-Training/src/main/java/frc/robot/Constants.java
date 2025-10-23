package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;

public class Constants {

  public static class SwerveConstants {

    public static final LinearVelocity MAX_LINEAR_VELOCITY =
        LinearVelocity.ofBaseUnits(4.25, MetersPerSecond);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY =
        AngularVelocity.ofBaseUnits(Math.PI * 2, RadiansPerSecond);

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(RobotConstants.ROBOT_LENGTH / 2, RobotConstants.ROBOT_WIDTH / 2),
            new Translation2d(RobotConstants.ROBOT_LENGTH / 2, -RobotConstants.ROBOT_WIDTH / 2),
            new Translation2d(-RobotConstants.ROBOT_LENGTH / 2, RobotConstants.ROBOT_WIDTH / 2),
            new Translation2d(-RobotConstants.ROBOT_WIDTH / 2, -RobotConstants.ROBOT_LENGTH / 2));
  }

  public static class RobotConstants {

    public static final double ROBOT_WIDTH = edu.wpi.first.math.util.Units.inchesToMeters(23.5);
    public static final double ROBOT_LENGTH = edu.wpi.first.math.util.Units.inchesToMeters(23.5);
  }
}
