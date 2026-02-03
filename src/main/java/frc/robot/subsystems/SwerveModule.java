package frc.robot.subsystems;

import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

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
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class SwerveModule extends SubsystemBase {
  
  private final SparkFlex driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkFlexConfig driveConfig;
  private final SparkClosedLoopController driveController;

  private final SparkMax turnMotor;
  private final AbsoluteEncoder turnEncoder;
  private final SparkMaxConfig turnConfig;
  private final SparkClosedLoopController turnController;
  
  private final Rotation2d offsetAngle;
  private SimpleMotorFeedforward turnFeedforward;

  private TrapezoidProfile profile;
  private TrapezoidProfile.State goalState;
  private TrapezoidProfile.State motorSetpoint;

  private LinearVelocity driveSetpointVelocity;
  private Current driveMotorCurrent;
  private Voltage driveMotorVoltage;
  private LinearVelocity driveMotorVelocity;
  private Distance driveMotorPosition;

  private Angle turnSetpointAngle;
  private Current turnMotorCurrent;
  private Voltage turnMotorVoltage;
  private AngularVelocity turnMotorVelocity;
  private Rotation2d turnMotorPosition;

  public SwerveModule(int driveMotorID, int TurnMotorID) {

    if (driveMotorID == SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID) {
      offsetAngle = new Rotation2d((Math.PI) / 2.0);
    } else if (driveMotorID == SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID) {
      offsetAngle = new Rotation2d(Math.PI);
    } else if (driveMotorID == SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID) {
      offsetAngle = new Rotation2d();
    } else {
      offsetAngle = new Rotation2d((3 * Math.PI) / 2.0);
    }

    driveMotor = new Spark(driveMotorID, MotorType.kBrushless);
    driveController = driveMotor.getClosedLoopController();
    driveEncoder = driveMotor.getEncoder();
    driveConfig = new SparkFlexConfig();
    driveConfig.smartCurrentLimit(60);
    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.inverted(false);
    
    driveConfig.encoder.positionConversionFactor(2 * Math.PI * .38);
    driveConfig.encoder.velocityConversionFactor(2 * Math.PI * .38 / 60);

    driveConfig.closedLoop.pidf(
      SwerveConstants.DRIVE_P,
      SwerveConstants.DRIVE_I,
      SwerveConstants.DRIVE_D,
      SwerveConstants.DRIVE_KV
    );

    turnMotor = new SparkMax(TurnMotorID, MotorType.kBrushless);
    turnController = turnMotor.getClosedLoopController();
    turnEncoder = turnMotor.getAbsoluteEncoder();
    turnConfig = new SparkMaxConfig();
    turnConfig.smartCurrentLimit(30);
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.inverted(false);
    
    turnConfig.absoluteEncoder.inverted(true);

    turnConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
    turnConfig.absoluteEncoder.velocityConversionFactor(2 * Math.PI);

    turnConfig.closedLoop.pid(
      SwerveConstants.TURN_P,
      SwerveConstants.TURN_I, 
      SwerveConstants.TURN_D
    );

    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    profile = new TrapezoidProfile(new Constraints(20 * Math.PI, 30 * Math.PI));

    goalState = new TrapezoidProfile.State(0, 0);
    motorSetpoint = new TrapezoidProfile.State(0, 0);

    turnFeedforward = new SimpleMotorFeedforward(
      SwerveConstants.TURN_KS, 
      SwerveConstants.TURN_KV
    );

    driveSetpointVelocity = LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond);
    
    driveMotorCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Units.Amps);
    driveMotorVoltage = Voltage.ofBaseUnits(driveMotor.getBusVoltage(), Units.Volts);
    driveMotorVelocity = LinearVelocity.ofBaseUnits(driveEncoder.getVelocity(), Units.MetersPerSecond);
    driveMotorPosition = Distance.ofBaseUnits(driveEncoder.getPosition(), Units.Meters);
    
    turnSetpointAngle = Units.Radians.of(0);
    
    turnMotorCurrent = Current.ofBaseUnits(turnMotor.getOutputCurrent(), Units.Amps);
    turnMotorVoltage = Voltage.ofBaseUnits(turnMotor.getBusVoltage(), Units.Volts);
    turnMotorVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), Units.RadiansPerSecond);
    turnMotorPosition = Rotation2d.fromRadians(turnEncoder.getPosition() - offsetAngle.getRadians());

  }

  public void setState(SwerveModuleState state) {
    state.optimize(turnMotorPosition);
    state.speedMetersPerSecond *= Math.cos(state.angle.getRadians() - turnMotorPosition.getRadians());

    driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

    driveSetpointVelocity = Units.MetersPerSecond.of(state.speedMetersPerSecond);
    turnSetpointAngle = Units.Radians.of(state.angle.getRadians());
  }

  @Override
  public void periodic() {
    driveMotorCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Units.Amps);
    driveMotorVoltage = Voltage.ofBaseUnits(driveMotor.getBusVoltage(), Units.Volts);
    driveMotorVelocity = LinearVelocity.ofBaseUnits(driveEncoder.getVelocity(), Units.MetersPerSecond);
    driveMotorPosition = Distance.ofBaseUnits(driveEncoder.getPosition(), Units.Meters);

    turnMotorCurrent = Current.ofBaseUnits(turnMotor.getOutputCurrent(), Units.Amps);
    turnMotorVoltage = Voltage.ofBaseUnits(turnMotor.getBusVoltage(), Units.Volts);
    turnMotorVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), Units.RadiansPerSecond);
    turnMotorPosition = Rotation2d.fromRadians(turnEncoder.getPosition() - offsetAngle.getRadians());

    goalState = new State(MathUtil.angleModulus(turnSetpointAngle.in(Units.Radians)) + offsetAngle.getRadians(), 0);

    motorSetpoint = profile.calculate(1 / RobotConstants.CLOCK, motorSetpoint, goalState);

    driveController.setReference(driveSetpointVelocity.in(Units.MetersPerSecond), ControlType.kVelocity);

    turnController.setReference(
      motorSetpoint.position,
      ControlType.kPosition,
      ClosedLoopSlot.kSlot0,
      turnFeedforward.calculate(motorSetpoint.velocity)
    );
  } 

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotorVelocity, turnMotorPosition);
  }

  public SwerveModulePosition getEstDrivePosition() {
    return new SwerveModulePosition(driveMotorPosition, turnMotorPosition);
  }
}

