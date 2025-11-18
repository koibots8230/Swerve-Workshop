package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.SwerveConstants;



public class SwerveModule{

    private Angle turnSetPointAngle;
    private LinearVelocity driveSetPointVelocity;
    double position;
    Rotation2d angle;

    @NotLogged private final SparkFlex driveMotor;
    @NotLogged private final SparkMax turnMotor;

    @NotLogged private final SparkFlexConfig driveConfig;
    @NotLogged private final SparkMaxConfig turnConfig;

    @NotLogged private final RelativeEncoder driveEncoder;
    @NotLogged private final AbsoluteEncoder turnEncoder;

    @NotLogged private final SparkClosedLoopController driveClosedLoopController;
    @NotLogged private final SparkClosedLoopController turnClosedLoopController;

    @NotLogged private final SimpleMotorFeedforward turnFeedforward;

    @NotLogged private final TrapezoidProfile turnProfile;
    private TrapezoidProfile.State turnGoalState;
    private TrapezoidProfile.State turnSetpointState;

    private Angle turnSetPoint;
    private LinearVelocity driveSetPoint;

    private final Rotation2d offset;

    double drivePosition;
    double turnPosition;
    double driveVelocity;
    private AngularVelocity turnVelocity;

    private Voltage turnVoltage;
    private double turnCurrent;
    private Voltage driveVoltage;
    private Current driveCurrent;

    public SwerveModule(int driveID, int turnID){

        if (driveID == SwerveConstants.FRONT_LEFT_DRIVE_ID){
            offset = SwerveConstants.OFFSETS[0];
        } else if (driveID == SwerveConstants.FRONT_RIGHT_DRIVE_ID){
            offset = SwerveConstants.OFFSETS[1];
        } else if (driveID == SwerveConstants.BACK_LEFT_DRIVE_ID){
            offset = SwerveConstants.OFFSETS[2];
        } else {
            offset = SwerveConstants.OFFSETS[3];
        }

        position = 0;
        angle = new Rotation2d();

        driveMotor = new SparkFlex(driveID, SparkFlex.MotorType.kBrushless);
        turnMotor = new SparkMax(turnID, SparkMax.MotorType.kBrushless);

        driveConfig = new SparkFlexConfig();

        driveConfig.closedLoop.pidf(
            SwerveConstants.DRIVE_PID.kp,
            SwerveConstants.DRIVE_PID.ki,
            SwerveConstants.DRIVE_PID.kd,
            SwerveConstants.DRIVE_FEEDFORWARD.kv
        );

        driveConfig.idleMode(IdleMode.kBrake);

        driveConfig.smartCurrentLimit((int) SwerveConstants.DRIVE_CURRENT_LIMIT.in(Amps));

        driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_CONVERSION_FACTOR);
        driveConfig.encoder.velocityConversionFactor(
            SwerveConstants.DRIVE_CONVERSION_FACTOR / 60.0);
        
        driveClosedLoopController = driveMotor.getClosedLoopController();

        turnConfig = new SparkMaxConfig();

        turnConfig.idleMode(IdleMode.kBrake);

        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(SwerveConstants.TURN_PID.kp, SwerveConstants.TURN_PID.ki, SwerveConstants.TURN_PID.kd)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI);

        turnConfig.smartCurrentLimit((int) SwerveConstants.TURN_CURRENT_LIMIT.in(Amps));

        turnConfig.absoluteEncoder.positionConversionFactor(SwerveConstants.TURN_CONVERSION_FACTOR);
        turnConfig.absoluteEncoder.velocityConversionFactor(
            SwerveConstants.TURN_CONVERSION_FACTOR / 60);
        turnConfig.absoluteEncoder.inverted(true);
        turnClosedLoopController = turnMotor.getClosedLoopController();

        turnProfile =
            new TrapezoidProfile(
             new TrapezoidProfile.Constraints(
                SwerveConstants.MAX_TURN_ACCELERATION, SwerveConstants.MAX_TURN_VELOCITY));

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder();

        turnFeedforward = new SimpleMotorFeedforward(SwerveConstants.TURN_FEEDFORWARD.ks, SwerveConstants.TURN_FEEDFORWARD.kv);

        driveSetPoint = LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond);
        turnSetPoint = Radians.of(0);
        drivePosition = driveEncoder.getPosition();
        turnPosition = turnEncoder.getPosition();
        driveVelocity = driveEncoder.getVelocity();
        turnVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), Units.RadiansPerSecond);
        driveVoltage = 
            Voltage.ofBaseUnits(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage(), Volts);
        turnVoltage =
            Voltage.ofBaseUnits(turnMotor.getAppliedOutput() * turnMotor.getBusVoltage(), Volts);
        driveCurrent =
            Current.ofBaseUnits(driveMotor.getOutputCurrent(), Amps);
        turnCurrent =
            turnMotor.getOutputCurrent();
    }

    public void setState(SwerveModuleState state){
        driveSetPointVelocity = MetersPerSecond.of(state.speedMetersPerSecond);
        turnSetPointAngle = Radians.of(state.angle.getRadians());
        position = (((driveSetPointVelocity.baseUnitMagnitude())));
        driveClosedLoopController.setReference(state.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
        
    }

    public void periodic(){
        drivePosition = driveEncoder.getPosition();
        driveVelocity = driveEncoder.getVelocity();
        driveVoltage = Voltage.ofBaseUnits(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput(), Volts);
        driveCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Amps);

        turnPosition = turnEncoder.getPosition() - offset.getRadians();
        turnVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), Units.RadiansPerSecond);
        turnVoltage = Voltage.ofBaseUnits(turnMotor.getBusVoltage() * turnMotor.getAppliedOutput(), Volts);
        turnCurrent = turnMotor.getOutputCurrent();

        turnGoalState = new TrapezoidProfile.State(MathUtil.angleModulus(turnSetPoint.in(Radians) + offset.getRadians()), 0);
        turnSetpointState = turnProfile.calculate(
            RobotConstants.CLOCK_SPEED.in(Seconds), 
            turnSetpointState,
            turnGoalState);

        turnClosedLoopController.setReference(
            turnSetpointState.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            turnFeedforward.calculate(turnSetpointState.velocity)
        );
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveSetPointVelocity, new Rotation2d(turnSetPointAngle));
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(position, angle);
    }
    
}
