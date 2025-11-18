package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.SwerveConstants;

import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {

  private Pose2d estimatedPose;
  private boolean isBlue;
  private Rotation2d simHeading;
  private ChassisSpeeds ChassisSpeeds;
  private SwerveModuleState[] moduleStates;
  private SwerveModuleState[] realModuleStates;
  private SwerveModulePosition[] modulePositions;
  private final Modules modules;
  private final SwerveDrivePoseEstimator swerveDriveEstimatedPose;

  public Swerve() {
    estimatedPose = new Pose2d();
    simHeading = new Rotation2d();
    ChassisSpeeds = new ChassisSpeeds();
    modules = new Modules();

    moduleStates = new SwerveModuleState[4];
    realModuleStates = new SwerveModuleState[4];
    modulePositions = new SwerveModulePosition[4];

    swerveDriveEstimatedPose = new SwerveDrivePoseEstimator(
      SwerveConstants.KINEMATICS, 
      simHeading, 
      getModulePositions(), 
      estimatedPose
    );
  }
  public class Modules{
    final SwerveModule frontLeftModule;
    final SwerveModule frontRightModule;
    final SwerveModule backLeftModule;
    final SwerveModule backRightModule;

    public Modules(){
      frontLeftModule = new SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE_ID, SwerveConstants.FRONT_LEFT_TURN_ID);
      frontRightModule = new SwerveModule(SwerveConstants.FRONT_RIGHT_DRIVE_ID, SwerveConstants.FRONT_RIGHT_TURN_ID);
      backLeftModule = new SwerveModule(SwerveConstants.BACK_LEFT_DRIVE_ID, SwerveConstants.BACK_LEFT_TURN_ID);
      backRightModule = new SwerveModule(SwerveConstants.BACK_RIGHT_DRIVE_ID, SwerveConstants.BACK_RIGHT_TURN_ID);
    }
  }

  public void setIsBlue(boolean color) {
    isBlue = color;
    simHeading = isBlue ? Rotation2d.fromRadians(0) : Rotation2d.fromRadians(Math.PI);
  }
  
  public void zeroGyro(boolean isBlue) {
   new Rotation2d();
  }

  public SwerveModulePosition[] getModulePositions(){
    modulePositions[0] = modules.frontLeftModule.getModulePosition();
    modulePositions[1] = modules.frontRightModule.getModulePosition();
    modulePositions[2] = modules.backLeftModule.getModulePosition();
    modulePositions[3] = modules.backRightModule.getModulePosition();
    return modulePositions;
  }

  private void fieldRelativeDrive(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    simHeading = new Rotation2d(-omega.baseUnitMagnitude()).div(50).plus(simHeading);
    swerveDriveEstimatedPose.update(simHeading,  modulePositions);
    estimatedPose = swerveDriveEstimatedPose.getEstimatedPosition();
    ChassisSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
      x.in(MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(), 
      y.in(MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(),
      omega.in(RotationsPerSecond) * SwerveConstants.MAX_ANGULAR_VELOCITY.baseUnitMagnitude(),
      simHeading);
    
    moduleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(ChassisSpeeds);

    modules.frontLeftModule.setState(moduleStates[0]);
    modules.frontRightModule.setState(moduleStates[1]);
    modules.backLeftModule.setState(moduleStates[2]);
    modules.backRightModule.setState(moduleStates[3]);

    realModuleStates[0] = modules.frontLeftModule.getState();
    realModuleStates[1] = modules.frontRightModule.getState();
    realModuleStates[2] = modules.backLeftModule.getState();
    realModuleStates[3] = modules.backRightModule.getState();
    
  }

  public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () ->
            fieldRelativeDrive(
                MetersPerSecond.of(x.getAsDouble()),
                MetersPerSecond.of(y.getAsDouble()),
                RotationsPerSecond.of(omega.getAsDouble())),
        this);
  }
}
