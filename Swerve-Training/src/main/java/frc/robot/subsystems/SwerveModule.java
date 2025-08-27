package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    private Angle turnSetpointAngle;
    private LinearVelocity driveSetpointVelocity;
    double position; 
    Rotation2d angle;
    
    public SwerveModule(){

        position = 0;
        angle = new Rotation2d();
        driveSetpointVelocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
        turnSetpointAngle = Radians.of(0);

    }

    public void setState(SwerveModuleState state){
        driveSetpointVelocity = MetersPerSecond.of(state.speedMetersPerSecond);
        turnSetpointAngle = Radians.of(state.angle.getRadians());
    }
    
    public void periodic(){

    }

    public void simulationPeriodic(){
        position = ((driveSetpointVelocity.baseUnitMagnitude() / RobotConstants.CLOCK) + position);
        angle = new Rotation2d(turnSetpointAngle);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(position, angle);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(position, angle);
    }
}