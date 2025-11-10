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

    public static final double TURN_KV = 0.55;
    public static final double TURN_KS = 0.0;
    public static final double DRIVE_KV = 0.18;
    public static final double DRIVE_KS = 0.0;

    public static final double TURN_P = 3;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0.0;

    public static final double DRIVE_P = .37;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_LEFT_TURN_MOTOR_ID = 2;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 4;
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
    public static final int BACK_LEFT_TURN_MOTOR_ID = 6;
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
    public static final int BACK_RIGHT_TURN_MOTOR_ID = 8;
  }

  public static class RobotConstants {

    public static final double ROBOT_WIDTH = edu.wpi.first.math.util.Units.inchesToMeters(23.5);
    public static final double ROBOT_LENGTH = edu.wpi.first.math.util.Units.inchesToMeters(23.5);
    public static final double CLOCK = 50;
  }
}
