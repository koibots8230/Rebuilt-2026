package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {
  private Pose2d estimatedPosition;
  private Rotation2d simHeading;
  private Rotation2d gyroAngle;
  private SwerveModuleState[] setpointStates;
  private final Pigeon2 gyro;

  private Rotation2d simHeading;

  @Logged
  public class Modules {
    final SwerveModule frontLeft;
    final SwerveModule frontRight;
    final SwerveModule backLeft;
    final SwerveModule backRight;

    public Modules() {
      frontLeftModule =
          new SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID, SwerveConstants.FRONT_LEFT_TURN_MOTOR_ID);
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

    gyro = new Pigeon2(SwerveConstants.GYRO_ID);

    estimatedPosition = new Pose2d();
    autoStartingPosition = new Translation2d();
    gyroAngle = gyro.getRotation2d().plus(Rotation2d.k180deg);
    simHeading = new Rotation2d();

    odometry =
        new SwerveDrivePoseEstimator(
            SwerveConstants.KINEMATICS, gyroAngle, this.getModulePostitions(), estimatedPosition);

    if (isReal) {
      System.out.println("Isreal triggered!!!");
      odometryUpdater =
          new Notifier(
              () -> {
                modules.frontLeft.updateLogs();
                modules.frontRight.updateLogs();
                modules.backLeft.updateLogs();
                modules.backRight.updateLogs();

                estimatedPosition =
                    odometry.updateWithTime(
                        Timer.getFPGATimestamp(),
                        isBlue ? gyro.getRotation2d() : gyro.getRotation2d().minus(Rotation2d.kPi),
                        getModulePostitions());
              });
      odometryUpdater.startPeriodic(0.005);
    }

    setpointStates = new SwerveModuleState[4];
    measuredStates = new SwerveModuleState[4];
  }

  public boolean getIsBlue() {
    return isBlue;
  }

  public void resetOdometry(Pose2d pose) {
    simHeading = pose.getRotation();
    odometry.resetPose(pose);
  }

  public void setIsBlue(boolean colour) {
    isBlue = colour;
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
                getChassisSpeeds().omegaRadiansPerSecond * RobotConstants.CLOCK_SPEED.in(Seconds)));
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

    modules.frontLeft.setState(setpointStates[0]);
    modules.frontRight.setState(setpointStates[1]);
    modules.backLeft.setState(setpointStates[2]);
    modules.backRight.setState(setpointStates[3]);
  }

  private void driveFieldRelative(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x.in(MetersPerSecond), y.in(MetersPerSecond), omega.in(RadiansPerSecond), gyroAngle);

    driveRobotRelative(speeds);
  }

  // ===================== Auto Driving ===================== \\

  private void followVector(LinearVelocity velocity, Rotation2d heading) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            heading.getCos() * velocity.in(MetersPerSecond),
            heading.getSin() * velocity.in(MetersPerSecond),
            0,
            gyroAngle);
    driveRobotRelative(speeds);
  }

  // ===================== Commands ===================== \\

  public Command autoDriveCommand(Distance distance, LinearVelocity velocity, Rotation2d heading) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              autoStartingPosition = estimatedPosition.getTranslation();
            }),
        Commands.race(
            Commands.run(() -> this.followVector(velocity, heading), this),
            Commands.waitUntil(
                () ->
                    estimatedPosition.getTranslation().getDistance(autoStartingPosition)
                        >= distance.in(Meters))),
        Commands.runOnce(() -> this.followVector(MetersPerSecond.of(0), Rotation2d.kZero), this));
  }

  public Command driveFieldRelativeCommand(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () ->
            fieldRelativeDrive(
                MetersPerSecond.of(
                    Math.pow(
                        MathUtil.applyDeadband(x.getAsDouble(), SwerveConstants.DEADBAND),
                        SwerveConstants.TRANSLATION_SCALING_EXPONENT)),
                MetersPerSecond.of(
                    Math.pow(
                        MathUtil.applyDeadband(y.getAsDouble(), SwerveConstants.DEADBAND),
                        SwerveConstants.TRANSLATION_SCALING_EXPONENT)),
                RotationsPerSecond.of(
                    Math.pow(
                        MathUtil.applyDeadband((omega.getAsDouble()), SwerveConstants.DEADBAND),
                        SwerveConstants.ROTATION_SCALING_EXPONENT))),
        this);
  }

  public Command zeroGyroCommand() {
    return Commands.runOnce(() -> zeroGyro(), this);
  }
}
