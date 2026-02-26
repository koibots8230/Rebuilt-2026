package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

@Logged
public class SwerveModule {
  @NotLogged private final SparkFlex driveMotor;
  @NotLogged private final SparkMax turnMotor;

  @NotLogged private final SparkMaxConfig turnConfig;
  @NotLogged private final SparkFlexConfig driveConfig;

  @NotLogged private final AbsoluteEncoder turnEncoder;
  @NotLogged private final RelativeEncoder driveEncoder;

  @NotLogged private final SparkClosedLoopController turnController;
  @NotLogged private final SparkClosedLoopController driveController;

  @NotLogged private final SimpleMotorFeedforward turnFeedforward;

  @NotLogged private final TrapezoidProfile turnProfile;
  private TrapezoidProfile.State turnGoalState;
  private TrapezoidProfile.State turnSetpointState;

  private final Rotation2d offset;

  private Angle turnSetpoint;
  private LinearVelocity driveSetpoint;

  double drivePosition;
  double turnPosition;
  double driveVelocity;
  private AngularVelocity turnVelocity;

  private Voltage turnVoltage;
  private Voltage driveVoltage;
  private double turnCurrent;
  private Current driveCurrent;

  public SwerveModule(int driveID, int turnID) {

    if (driveID == SwerveConstants.FRONT_LEFT_DRIVE_ID) {
      offset = SwerveConstants.OFFSETS[0];
    } else if (driveID == SwerveConstants.FRONT_RIGHT_DRIVE_ID) {
      offset = SwerveConstants.OFFSETS[1];
    } else if (driveID == SwerveConstants.BACK_LEFT_DRIVE_ID) {
      offset = SwerveConstants.OFFSETS[2];
    } else {
      offset = SwerveConstants.OFFSETS[3];
    }

    turnProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                SwerveConstants.MAX_TURN_VELOCITY.in(RadiansPerSecond),
                SwerveConstants.MAX_TURN_ACCELRATION.in(RadiansPerSecondPerSecond)));

    turnGoalState = new TrapezoidProfile.State(0, 0);
    turnSetpointState = new TrapezoidProfile.State(0, 0);

    turnMotor = new SparkMax(turnID, MotorType.kBrushless);
    driveMotor = new SparkFlex(driveID, MotorType.kBrushless);

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

    driveConfig = new SparkFlexConfig();
    driveConfig.smartCurrentLimit((int) SwerveConstants.DRIVE_CURRENT_LIMIT.in(Units.Amps));
    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.inverted(false);

    driveConfig.encoder.positionConversionFactor(2 * Math.PI);
    driveConfig.encoder.velocityConversionFactor(2 * Math.PI / 60);

    driveConfig.closedLoop.pid(
        SwerveConstants.DRIVE_PID.kp,
        SwerveConstants.DRIVE_PID.ki,
        SwerveConstants.DRIVE_PID.kd,
        ClosedLoopSlot.kSlot0);

    turnMotor = new SparkMax(TurnMotorID, SparkLowLevel.MotorType.kBrushless);
    turnController = turnMotor.getClosedLoopController();
    turnEncoder = turnMotor.getAbsoluteEncoder();
    turnConfig = new SparkMaxConfig();
    turnConfig.smartCurrentLimit((int) SwerveConstants.TURN_CURRENT_LIMIT.in(Units.Amps));
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.inverted(false);

    turnConfig.absoluteEncoder.inverted(true);

    turnConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
    turnConfig.absoluteEncoder.velocityConversionFactor(2 * Math.PI / 60);

    turnConfig.closedLoop.pid(
        SwerveConstants.TURN_PID.kp, SwerveConstants.TURN_PID.ki, SwerveConstants.TURN_PID.kd);

    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnEncoder = turnMotor.getAbsoluteEncoder();
    turnController = turnMotor.getClosedLoopController();

    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();

    turnFeedforward =
        new SimpleMotorFeedforward(
            SwerveConstants.TURN_FEEDFORWARD.ks, SwerveConstants.TURN_FEEDFORWARD.kv);

    driveSetpointVelocity = Units.MetersPerSecond.of(0);

    driveMotorPosition = Units.Meters.of(0);
    driveMotorCurrent = Units.Amps.of(driveMotor.getOutputCurrent());
    driveMotorVoltage = Units.Volts.of(driveMotor.getBusVoltage());
    driveMotorVelocity = Units.MetersPerSecond.of(0);
    simDrivePosition = Units.Meters.of(0);

    turnSetpointAngle = Units.Radians.of(0);

    turnMotorPosition = Rotation2d.fromRadians(turnEncoder.getPosition());
    turnMotorCurrent = Units.Amps.of(turnMotor.getOutputCurrent());
    turnMotorVoltage = Units.Volts.of(turnMotor.getBusVoltage());
    turnMotorVelocity = Units.RadiansPerSecond.of(turnEncoder.getVelocity());
  }

  public void setState(SwerveModuleState swerveModuleState) {
    swerveModuleState.optimize(Rotation2d.fromRadians(MathUtil.angleModulus(turnPosition)));

    swerveModuleState.speedMetersPerSecond *=
        Math.cos(swerveModuleState.angle.getRadians() - turnPosition);

    driveController.setSetpoint(
        swerveModuleState.speedMetersPerSecond, SparkBase.ControlType.kVelocity);

    driveSetpoint = MetersPerSecond.of(swerveModuleState.speedMetersPerSecond);
    turnSetpoint = Radians.of(swerveModuleState.angle.getRadians());
  }

  public void updateLogs() {
    driveCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Units.Amps);
    turnCurrent = turnMotor.getOutputCurrent();
    driveVoltage =
        Voltage.ofBaseUnits(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput(), Volts);
    turnVoltage =
        Voltage.ofBaseUnits(turnMotor.getBusVoltage() * turnMotor.getAppliedOutput(), Volts);
    drivePosition = driveEncoder.getPosition();
    turnPosition = turnEncoder.getPosition() - offset.getRadians();
    driveVelocity = driveEncoder.getVelocity();
    turnVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), Units.RadiansPerSecond);
  }

  public void periodic() {
    turnGoalState =
        new TrapezoidProfile.State(
            MathUtil.angleModulus(turnSetpoint.in(Radians)) + offset.getRadians(), 0);

    motorSetpoint =
        profile.calculate(RobotConstants.CLOCK_SPEED.in(Milliseconds), motorSetpoint, goalState);

    turnController.setSetpoint(
        turnSetpointState.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        turnFeedforward.calculate(turnSetpointState.velocity));
  }

  public void simulationPeriodic() {

    simDrivePosition =
        simDrivePosition.plus(driveSetpointVelocity.times(RobotConstants.CLOCK_SPEED));
    driveMotorPosition = simDrivePosition;
    turnMotorPosition = new Rotation2d(turnSetpointAngle);
    driveMotorVelocity = driveSetpointVelocity;
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(driveVelocity, Rotation2d.fromRadians(turnPosition));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(drivePosition, Rotation2d.fromRadians(turnPosition));
  }
}
