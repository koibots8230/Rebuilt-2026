package frc.robot.subsystems;

public class swerve extends SubsystemBase {

  private boolean isBlue;
  private Pose2d odometryPose;
  private SwerveModuleState[] setpointStates;
  private SwerveModuleState[] messuredModuleStates;
  private ChassisSpeeds chassisSpeeds;
  private Rotation2d gyroAngle;
  
  private final SwerveDrivePoseEstimator odometry;
  private final Pigeon2 gyro;
  private final Modules modules;
  
  public class Modules {
    final SwerveModule frontLeftModule;
    final SwerveModule frontRightModule;
    final SwerveModule backLeftModule;
    final SwerveModule backRightModule;

    public Modules() {
      frontLeftModule = new SwerveModule(
        SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID, 
        SwerveConstants.FRONT_LEFT_TURN_MOTOR_ID
      );
      frontRightModule = new SwerveModule(
        SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
        SwerveConstants.FRONT_RIGHT_TURN_MOTOR_ID
      );
      backLeftModule = new SwerveModule(
        SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID, 
        SwerveConstants.BACK_LEFT_TURN_MOTOR_ID
      );
      new SwerveModule(
        SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_ID, 
        SwerveConstants.BACK_RIGHT_TURN_MOTOR_ID
      );
    }
  }

  public swerve() {
    gyro = new Pigeon2(10);
    modules = new Modules();

    odometryPose = new Pose2d();
    gyroAngle = new gyro.getRotation2d();
    chassisSpeeds = new ChassisSpeeds();

    setpointStates = new SwerveModuleState[4];
    messuredModuleStates = new SwerveModuleState[4];

    odometry = new SwerveDrivePoseEstimator(
      SwerveConstants.KINEMATICS, 
      gyroAngle, 
      modulePosition(), 
      odometryPose
    );

  }

  public void setIsBlue(Boolean allianceColour) {
    isBlue = allianceColor;
    simHeading = (isBlue ? new Rotation2d() : new Rotation2d(Math.PI));
    estimatedPose = new Pose2d(0, 0, simHeading);
  }

  @Override
  public void periodic() {
    modules.frontLeftModule.periodic();
    modules.frontRightModule.periodic();
    modules.backLeftModule.periodic();
    modules.backRightModule.periodic();

    messuredModuleStates[0] = modules.frontLeftModule.getState();
    messuredModuleStates[1] = modules.frontRightModule.getState();
    messuredModuleStates[2] = modules.backLeftModule.getState();
    messuredModuleStates[3] = modules.backRightModule.getState();

    odometryPose = odometry.update(
      isBlue ? gyroAngle : gyroAngle.minus(new Rotation2d(Math.PI)), 
      modulePosition()
    );

    gyroAngle = gyro.getRotation2d();
  }

  private void fieldRelitiveDrive(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      x.in(MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(),
      y.in(MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(),
      omega.in(RotationsPerSecond) * SwerveConstants.MAX_ANGULAR_VELOCITY.baseUnitMagnitude(),
      gyroAngle
    );

  setpointStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

  SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_LINEAR_VELOCITY);

  modules.frontLeftModule.setState(setpointStates[0]);
  modules.frontRightModule.setState(setpointStates[1]);
  modules.backLeftModule.setState(setpointStates[2]);
  modules.backRightModule.setState(setpointStates[3]);
  
  }

  private SwerveModulePosition[] modulePosition() {
    return new SwerveModulePosition[] {
      modules.frontLeftModule.getSimDrivePosition(),
      modules.frontRightModule.getSimDrivePosition(),
      modules.backLeftModule.getSimDrivePosition(),
      modules.backRightModule.getSimDrivePosition()
    };
  }

  public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
      () -> fieldRelitiveDrive(
        MetersPerSecond.of(MathUtil.applyDeadband(x.getAsDouble(), 0.07)),
        MetersPerSecond.of(MathUtil.applyDeadband(y.getAsDouble(), 0.07)),
        RotationsPerSecond.of(MathUtil.applyDeadband((omega.getAsDouble()), 0.07))
      ), this
    );
  }

}