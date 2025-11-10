package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {

  private boolean isBlue;
  private Pose2d estimatedPose;
  private SwerveModuleState[] moduleStates;
  private SwerveModuleState[] messuredModuleStates;
  private ChassisSpeeds chassisSpeeds;
  private Rotation2d simHeading;
  private Rotation2d gyroAngle;
  private final Pigeon2 gyro;
  @NotLogged private final Modules modules;
  @NotLogged private final SwerveDrivePoseEstimator odometry;

  public class Modules {
    final SwerveModule frontLeftModule;
    final SwerveModule frontRightModule;
    final SwerveModule backLeftModule;
    final SwerveModule backRightModule;

    public Modules() {
      frontLeftModule =
          new SwerveModule(
              SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID, SwerveConstants.FRONT_LEFT_TURN_MOTOR_ID);
      frontRightModule =
          new SwerveModule(
              SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
              SwerveConstants.FRONT_RIGHT_TURN_MOTOR_ID);
      backLeftModule =
          new SwerveModule(
              SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID, SwerveConstants.BACK_LEFT_TURN_MOTOR_ID);
      backRightModule =
          new SwerveModule(
              SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_ID, SwerveConstants.BACK_RIGHT_TURN_MOTOR_ID);
    }
  }

  public Swerve() {

    gyro = new Pigeon2(9);

    modules = new Modules();

    estimatedPose = new Pose2d();
    simHeading = new Rotation2d();
    gyroAngle = gyro.getRotation2d();
    chassisSpeeds = new ChassisSpeeds();

    moduleStates = new SwerveModuleState[4];
    messuredModuleStates = new SwerveModuleState[4];

    odometry =
        new SwerveDrivePoseEstimator(
            SwerveConstants.KINEMATICS, gyroAngle, modulePosition(), estimatedPose);
  }

  public void setIsBlue(Boolean allianceColour) {
    isBlue = allianceColour;
    simHeading = (isBlue ? new Rotation2d() : new Rotation2d(Math.PI));
    estimatedPose = new Pose2d(0, 0, simHeading);
  }

  public void zeroSimGyro() {
    simHeading = new Rotation2d();
  }

  @Override
  public void periodic() {

    modules.frontLeftModule.setState(moduleStates[0]);
    modules.frontRightModule.setState(moduleStates[1]);
    modules.backLeftModule.setState(moduleStates[2]);
    modules.backRightModule.setState(moduleStates[3]);

    messuredModuleStates[0] = modules.frontLeftModule.getState();
    messuredModuleStates[1] = modules.frontRightModule.getState();
    messuredModuleStates[2] = modules.backLeftModule.getState();
    messuredModuleStates[3] = modules.backRightModule.getState();

    estimatedPose = odometry.update(simHeading, this.modulePosition());
  }

  @Override
  public void simulationPeriodic() {
    simHeading = simHeading.plus(new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * 0.02));
    gyroAngle = simHeading;

    modules.frontLeftModule.simulationPeriodic();
    modules.frontRightModule.simulationPeriodic();
    modules.backLeftModule.simulationPeriodic();
    modules.backRightModule.simulationPeriodic();
  }

  private void fieldRelitiveDrive(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x.in(MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(),
            y.in(MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(),
            omega.in(RotationsPerSecond) * SwerveConstants.MAX_ANGULAR_VELOCITY.baseUnitMagnitude(),
            simHeading);
    moduleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
  }

  private SwerveModulePosition[] modulePosition() {
    return new SwerveModulePosition[] {
      modules.frontLeftModule.getPosition(),
      modules.frontRightModule.getPosition(),
      modules.backLeftModule.getPosition(),
      modules.backRightModule.getPosition()
    };
  }

  public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () ->
            fieldRelitiveDrive(
                MetersPerSecond.of(MathUtil.applyDeadband(x.getAsDouble(), 0.07)),
                MetersPerSecond.of(MathUtil.applyDeadband(y.getAsDouble(), 0.07)),
                RotationsPerSecond.of(MathUtil.applyDeadband((omega.getAsDouble()), 0.07))),
                this);
  }
}
