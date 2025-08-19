package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;;

@Logged
public class Swerve extends SubsystemBase {

  private boolean isBlue;
  private Pose2d estimatedPose;
  private SwerveModuleState[] moduleStates;
  private SwerveModuleState[] realModuleStates;
  private ChassisSpeeds chassisSpeeds;  
  private Rotation2d simHeading;
  private final Modules modules;
  
    public class Modules{
      final SwerveModule frontLeftModule;
      final SwerveModule frontRightModule;
      final SwerveModule backLeftModule;
      final SwerveModule backRightModule;


      public Modules(){
        frontLeftModule = new SwerveModule();
        frontRightModule = new SwerveModule();
        backLeftModule = new SwerveModule();
        backRightModule = new SwerveModule();
        }

    }

    

    public Swerve(){

      modules = new Modules();

      estimatedPose = new Pose2d();
      simHeading = new Rotation2d();
      chassisSpeeds = new ChassisSpeeds();

      moduleStates = new SwerveModuleState[4];
      realModuleStates = new SwerveModuleState[4];
    }

    public void setIsBlue(Boolean allianceColour){
      isBlue = allianceColour;
      simHeading = (isBlue ? new Rotation2d() : new Rotation2d(Math.PI));
      estimatedPose = new Pose2d(0,0,simHeading);
      }

    public void ZeroSimGyro(){
      simHeading = new Rotation2d();
    }

    private void fieldRelitiveDrive(LinearVelocity x, LinearVelocity y, AngularVelocity omega){
      simHeading = new Rotation2d(omega.times(Second.of(.02)));
      estimatedPose = new Pose2d(((x.baseUnitMagnitude() / 20) * (isBlue ? -1 : 1) + estimatedPose.getX()), (y.baseUnitMagnitude() / 20) * (isBlue ? -1 : 1) + estimatedPose.getY(), simHeading.plus(new Rotation2d(estimatedPose.getRotation().getRadians())));
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x.in(MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(), y.in(MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(), omega.in(RotationsPerSecond) * SwerveConstants.MAX_ANGULAR_VELOCITY.baseUnitMagnitude(), simHeading);
      moduleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      modules.frontLeftModule.setState(moduleStates[0]);
      modules.frontRightModule.setState(moduleStates[1]);
      modules.backLeftModule.setState(moduleStates[2]);
      modules.backRightModule.setState(moduleStates[3]);
      
      realModuleStates[0] = modules.frontLeftModule.getState();
      realModuleStates[1] = modules.frontRightModule.getState();
      realModuleStates[2] = modules.backLeftModule.getState();
      realModuleStates[3] = modules.backRightModule.getState();
    }

    public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega){
      return Commands.run(() -> fieldRelitiveDrive(MetersPerSecond.of(x.getAsDouble()), MetersPerSecond.of(y.getAsDouble()), RotationsPerSecond.of(-omega.getAsDouble())), this);
    }
}
