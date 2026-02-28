package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {

  private boolean isBlue;
  private Pose2d odometryPose;
  private SwerveModuleState[] setpointStates;
  private SwerveModuleState[] messuredModuleStates;
  private ChassisSpeeds chassisSpeeds;
  private Rotation2d gyroAngle;

  private final SwerveDrivePoseEstimator odometry;
  private final Pigeon2 gyro;

  private Rotation2d simHeading;

  @Logged
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

  private final Modules modules;

  public Swerve() {
    gyro = new Pigeon2(SwerveConstants.GYRO_ID);
    modules = new Modules();

    odometryPose = new Pose2d();
    gyroAngle = gyro.getRotation2d();
    chassisSpeeds = new ChassisSpeeds();

    setpointStates = new SwerveModuleState[4];
    messuredModuleStates = new SwerveModuleState[4];

    simHeading = new Rotation2d();

    odometry =
        new SwerveDrivePoseEstimator(
            SwerveConstants.KINEMATICS, gyroAngle, modulePosition(), odometryPose);
  }

  public void setIsBlue(Boolean allianceColor) {
    isBlue = allianceColor;
    Rotation2d estHeading = (isBlue ? new Rotation2d() : new Rotation2d(Math.PI));
    odometryPose = new Pose2d(0, 0, estHeading);
  }

  @Override
  public void periodic() {
    odometryPose =
        odometry.update(isBlue ? gyroAngle : gyroAngle.minus(Rotation2d.kPi), modulePosition());

    modules.frontLeftModule.periodic();
    modules.frontRightModule.periodic();
    modules.backLeftModule.periodic();
    modules.backRightModule.periodic();

    messuredModuleStates[0] = modules.frontLeftModule.getState();
    messuredModuleStates[1] = modules.frontRightModule.getState();
    messuredModuleStates[2] = modules.backLeftModule.getState();
    messuredModuleStates[3] = modules.backRightModule.getState();

    gyroAngle = gyro.getRotation2d();
  }

  @Override
  public void simulationPeriodic() {
    modules.frontLeftModule.simulationPeriodic();
    modules.frontRightModule.simulationPeriodic();
    modules.backLeftModule.simulationPeriodic();
    modules.backRightModule.simulationPeriodic();

    messuredModuleStates[0] = setpointStates[0];
    messuredModuleStates[1] = setpointStates[1];
    messuredModuleStates[2] = setpointStates[2];
    messuredModuleStates[3] = setpointStates[3];

    simHeading =
        simHeading.plus(
            new Rotation2d(
                chassisSpeeds.omegaRadiansPerSecond
                    * RobotConstants.CLOCK_SPEED.baseUnitMagnitude()));
    gyroAngle = simHeading;
  }

  private void fieldRelativeDrive(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x.in(Units.MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(),
            y.in(Units.MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(),
            omega.in(Units.RotationsPerSecond)
                * SwerveConstants.MAX_ANGULAR_VELOCITY.baseUnitMagnitude(),
            gyroAngle);

    chassisSpeeds =
        ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.CLOCK_SPEED.in(Units.Seconds));

    setpointStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, SwerveConstants.MAX_LINEAR_VELOCITY);

    modules.frontLeftModule.setState(setpointStates[0]);
    modules.frontRightModule.setState(setpointStates[1]);
    modules.backLeftModule.setState(setpointStates[2]);
    modules.backRightModule.setState(setpointStates[3]);
  }

  private SwerveModulePosition[] modulePosition() {
    return new SwerveModulePosition[] {
      modules.frontLeftModule.getEstDrivePosition(),
      modules.frontRightModule.getEstDrivePosition(),
      modules.backLeftModule.getEstDrivePosition(),
      modules.backRightModule.getEstDrivePosition()
    };
  }

  public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () ->
            fieldRelativeDrive(
                MetersPerSecond.of(MathUtil.applyDeadband(x.getAsDouble(), SwerveConstants.DEADBAND)),
                MetersPerSecond.of(MathUtil.applyDeadband(y.getAsDouble(), SwerveConstants.DEADBAND)),
                RotationsPerSecond.of(MathUtil.applyDeadband((omega.getAsDouble()), SwerveConstants.DEADBAND))),
        this);
  }
}
