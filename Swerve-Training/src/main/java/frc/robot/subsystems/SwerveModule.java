package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

@Logged
public class SwerveModule {

  @NotLogged private final SparkMax turnMotor;
  @NotLogged private final SparkFlex driveMotor;

  @NotLogged private final AbsoluteEncoder turnEncoder;
  @NotLogged private final RelativeEncoder driveEncoder;

  @NotLogged private final SparkMaxConfig turnConfig;
  @NotLogged private final SparkFlexConfig driveConfig;

  @NotLogged private final SparkClosedLoopController turnController;
  @NotLogged private final SparkClosedLoopController driveController;

  @NotLogged private final TrapezoidProfile profile;
  private TrapezoidProfile.State goalState;
  private TrapezoidProfile.State motorSetpoint;

  @NotLogged private final SimpleMotorFeedforward turnFeedforward;

  private Angle turnSetpointAngle;
  private LinearVelocity driveSetpointVelocity;
  double simDrivePosition;
  private final Rotation2d offsetAngle;

  private Current driveMotorCurrent;
  private Current turnMotorCurrent;

  private Voltage driveMotorVoltage;
  private Voltage turnMotorVoltage;

  private double driveMotorVelocity;
  private AngularVelocity turnMotorVelocity;

  private double driveMotorPosition;
  private Rotation2d turnMotorPosition;

  public SwerveModule(int driveMotorID, int TurnMotorID) {

    // if (driveMotorID == SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID) {
    //   offsetAngle = new Rotation2d((3 * Math.PI) / 2);
    // } else if (driveMotorID == SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID) {
    //   offsetAngle = new Rotation2d();
    // } else if (driveMotorID == SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID) {
    //   offsetAngle = new Rotation2d(Math.PI);
    // } else {
    //   offsetAngle = new Rotation2d(Math.PI / 2);
    // }

    if (driveMotorID == SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID) {
      offsetAngle = new Rotation2d((Math.PI) / 2.0);
    } else if (driveMotorID == SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID) {
      offsetAngle = new Rotation2d(Math.PI);
    } else if (driveMotorID == SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID) {
      offsetAngle = new Rotation2d();
    } else {
      offsetAngle = new Rotation2d((3 * Math.PI) / 2.0);
    }

    turnMotor = new SparkMax(TurnMotorID, MotorType.kBrushless);
    driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);

    turnEncoder = turnMotor.getAbsoluteEncoder();
    driveEncoder = driveMotor.getEncoder();

    turnController = turnMotor.getClosedLoopController();
    driveController = driveMotor.getClosedLoopController();

    turnConfig = new SparkMaxConfig();
    driveConfig = new SparkFlexConfig();

    turnConfig.idleMode(IdleMode.kBrake);
    driveConfig.idleMode(IdleMode.kBrake);

    turnConfig.smartCurrentLimit(30);
    driveConfig.smartCurrentLimit(60);

    turnConfig.inverted(false);
    driveConfig.inverted(false);

    turnConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
    driveConfig.encoder.positionConversionFactor(2 * Math.PI * .38);

    turnConfig.absoluteEncoder.velocityConversionFactor(2 * Math.PI);
    driveConfig.encoder.velocityConversionFactor(2 * Math.PI * .38 / 60);

    turnConfig.closedLoop.pid(
        SwerveConstants.TURN_P, SwerveConstants.TURN_I, SwerveConstants.TURN_D);
    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);

    driveConfig.closedLoop.pidf(
        SwerveConstants.DRIVE_P,
        SwerveConstants.DRIVE_I,
        SwerveConstants.DRIVE_D,
        SwerveConstants.DRIVE_KV);

    turnConfig.absoluteEncoder.inverted(true);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    profile = new TrapezoidProfile(new Constraints(20 * Math.PI, 30 * Math.PI));

    goalState = new TrapezoidProfile.State(0, 0);
    motorSetpoint = new TrapezoidProfile.State(0, 0);

    turnFeedforward =
        new SimpleMotorFeedforward(
            Constants.SwerveConstants.TURN_KS, Constants.SwerveConstants.TURN_KV);

    simDrivePosition = 0;
    driveSetpointVelocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
    turnSetpointAngle = Radians.of(0);

    driveMotorCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Amps);
    turnMotorCurrent = Current.ofBaseUnits(turnMotor.getOutputCurrent(), Amps);
    driveMotorVoltage = Voltage.ofBaseUnits(driveMotor.getBusVoltage(), Volts);
    turnMotorVoltage = Voltage.ofBaseUnits(turnMotor.getBusVoltage(), Volts);
    driveMotorVelocity = driveEncoder.getVelocity();
    turnMotorVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), RadiansPerSecond);
    driveMotorPosition = driveEncoder.getPosition();
    turnMotorPosition = Rotation2d.fromRadians(turnEncoder.getPosition());
  }

  public void setState(SwerveModuleState state) {
    state.optimize(Rotation2d.fromRadians(MathUtil.angleModulus(turnMotorPosition.getRadians())));
    state.speedMetersPerSecond *= Math.cos(state.angle.getRadians() - turnMotorPosition.getRadians());

    driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

    driveSetpointVelocity = MetersPerSecond.of(state.speedMetersPerSecond);
    turnSetpointAngle = Radians.of(state.angle.getRadians());
  }

  public void periodic() {

    driveMotorCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Amps);
    turnMotorCurrent = Current.ofBaseUnits(turnMotor.getOutputCurrent(), Amps);
    driveMotorVoltage = Voltage.ofBaseUnits(driveMotor.getBusVoltage(), Volts);
    turnMotorVoltage = Voltage.ofBaseUnits(turnMotor.getBusVoltage(), Volts);
    driveMotorVelocity = driveEncoder.getVelocity();
    turnMotorVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), RadiansPerSecond);
    driveMotorPosition = driveEncoder.getPosition();
    turnMotorPosition = Rotation2d.fromRadians(turnEncoder.getPosition() - offsetAngle.getRadians());

    goalState = new State(MathUtil.angleModulus(turnSetpointAngle.in(Radians)) + offsetAngle.getRadians(), 0);

    motorSetpoint = profile.calculate(1 / RobotConstants.CLOCK, motorSetpoint, goalState);

    turnController.setReference(
        motorSetpoint.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        turnFeedforward.calculate(motorSetpoint.velocity));
  }

  public void simulationPeriodic() {
    driveMotorPosition = ((driveSetpointVelocity.baseUnitMagnitude() / RobotConstants.CLOCK) + simDrivePosition);
    turnMotorPosition = new Rotation2d(turnSetpointAngle);
    driveMotorVelocity = driveSetpointVelocity.in(MetersPerSecond);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotorVelocity, turnMotorPosition);
  }

  public SwerveModulePosition getSimDrivePosition() {
    return new SwerveModulePosition(driveMotorPosition, turnMotorPosition);
  }
}
