package frc.robot.subsystems;

public class SwerveModule extends SubsystemBase {
  
  private final SparkMax turnMotor;
  private final SparkMax driveMotor;
  private final AbsoluteEncoder turnEncoder;
  private final RelativweEncoder driveEncoder;
  private final SparkMaxConfig turnConfig;
  private final SparkMaxConfig driveConfig;
  private final SparkClosedLoopController turnController;
  private final SparkClosedLoopController driveController;
  
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

    driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);
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
      Constants.SwerveConstants.TURN_KS, 
      Constants.SwerveConstants.TURN_KV
    );

    driveSetpointVelocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
    
    driveMotorCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Amps);
    driveMotorVoltage = Voltage.ofBaseUnits(driveMotor.getBusVoltage(), Volts);
    driveMotorVelocity = driveEncoder.getVelocity();
    driveMotorPosition = driveEncoder.getPosition();
    
    turnSetpointAngle = Radians.of(0);
    
    turnMotorCurrent = Current.ofBaseUnits(turnMotor.getOutputCurrent(), Amps);
    turnMotorVoltage = Voltage.ofBaseUnits(turnMotor.getBusVoltage(), Volts);
    turnMotorVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), RadiansPerSecond);
    turnMotorPosition = Rotation2d.fromRadians(turnEncoder.getPosition() - offsetAngle.getRadians());

  }

  public void setState(SwerveModuleState state) {

  }

  @Override
  public void periodic() {

  } 

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotorVelocity, turnMotorPosition);
  }

  public SwerveModulePosition getSimDrivePosition() {
    return new SwerveModulePosition(driveMotorPosition, turnMotorPosition);
  }
}

