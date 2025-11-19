package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public class Constants {
  
  public static class RobotConstants {

    public static final int CONTROLLER_PORT = 0;

    public static final double ROBOT_LENGTH = edu.wpi.first.math.util.Units.inchesToMeters(21.375);
    public static final double ROBOT_WIDTH = edu.wpi.first.math.util.Units.inchesToMeters(21.375);

    public static final Time CLOCK_SPEED = Seconds.of(0.02); // 20ms
    
    public static class SwerveConstants{
      
      public static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(4);
      public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(2 * Math.PI); 
      public static final SwerveDriveKinematics KINEMATICS = 
      new SwerveDriveKinematics(
        new Translation2d(RobotConstants.ROBOT_LENGTH / 2, RobotConstants.ROBOT_WIDTH / 2),
        new Translation2d(RobotConstants.ROBOT_LENGTH / 2, -RobotConstants.ROBOT_WIDTH / 2),
        new Translation2d(-RobotConstants.ROBOT_LENGTH / 2, RobotConstants.ROBOT_WIDTH / 2),
        new Translation2d(-RobotConstants.ROBOT_LENGTH / 2, -RobotConstants.ROBOT_WIDTH / 2)
      );

      public static final double MAX_TURN_ACCELERATION = 30 * Math.PI;
      public static final double MAX_TURN_VELOCITY = 20 * Math.PI;

      public static final PIDGains DRIVE_PID = 
        new PIDGains.Builder().kp(0).ki(0).kd(0).build();
      public static final FeedforwardGains DRIVE_FEEDFORWARD = 
        new FeedforwardGains.Builder().kv(0).build();

      public static final PIDGains TURN_PID = new PIDGains.Builder().kp(0).ki(0).kd(0).build();
      public static final FeedforwardGains TURN_FEEDFORWARD = new FeedforwardGains.Builder().kv(0).build();

      public static final Current DRIVE_CURRENT_LIMIT = Current.ofBaseUnits(30, Amps);
      public static final Current TURN_CURRENT_LIMIT = Current.ofBaseUnits(60, Amps);

      public static final double SWERVE_GEARING = 5.50;

      public static final double DRIVE_CONVERSION_FACTOR = 
      (Units.inchesToMeters(1.5) * 2 * Math.PI / SWERVE_GEARING);
      public static final double TURN_CONVERSION_FACTOR = 2 * Math.PI;


      public static final Rotation2d[] OFFSETS = {
        Rotation2d.fromRadians(0), // Front Left
        Rotation2d.fromRadians(0), // Front Right
        Rotation2d.fromRadians(0), // Back Left
        Rotation2d.fromRadians(0)  // Back Right
      };

      public static final int FRONT_LEFT_DRIVE_ID = 1;
      public static final int FRONT_LEFT_TURN_ID = 2;

      public static final int FRONT_RIGHT_DRIVE_ID = 3;
      public static final int FRONT_RIGHT_TURN_ID = 4;

      public static final int BACK_LEFT_DRIVE_ID = 5;
      public static final int BACK_LEFT_TURN_ID = 6;

      public static final int BACK_RIGHT_DRIVE_ID = 7;
      public static final int BACK_RIGHT_TURN_ID = 8;

      public static final int GYRO_ID = 9;
    }
  }
}