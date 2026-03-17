package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.VisionMeasurement;
import frc.robot.Constants.*;
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {
  private Pose2d estimatedPosition;
  private Rotation2d simHeading;
  private Rotation2d gyroAngle;
  private SwerveModuleState[] setpointStates;
  private final Pigeon2 gyro;

  private final PIDController xController =
      new PIDController(AutoConstants.X_CONTROLLER.kp, 0.0, 0.0);
  private final PIDController yController =
      new PIDController(AutoConstants.Y_CONTROLLER.kp, 0.0, 0.0);
  private final PIDController headingController =
      new PIDController(AutoConstants.OMEGA_CONTROLLER.kp, 0.0, 0.0);

  @Logged
  public class Modules {
    final SwerveModule frontLeft;
    final SwerveModule frontRight;
    final SwerveModule backLeft;
    final SwerveModule backRight;

    public Modules() {
      frontLeft =
          new SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE_ID, SwerveConstants.FRONT_LEFT_TURN_ID);
      frontRight =
          new SwerveModule(
              SwerveConstants.FRONT_RIGHT_DRIVE_ID, SwerveConstants.FRONT_RIGHT_TURN_ID);
      backLeft =
          new SwerveModule(SwerveConstants.BACK_LEFT_DRIVE_ID, SwerveConstants.BACK_LEFT_TURN_ID);
      backRight =
          new SwerveModule(SwerveConstants.BACK_RIGHT_DRIVE_ID, SwerveConstants.BACK_RIGHT_TURN_ID);
    }
  }

  private final Modules modules;

  @NotLogged private final SwerveDrivePoseEstimator odometry;
  @NotLogged private Notifier odometryUpdater;

  private SwerveModuleState[] measuredStates;

  private boolean isBlue;

  private Translation2d autoStartingPosition;

  public Swerve(boolean isReal) {

    modules = new Modules();

    gyro = new Pigeon2(SwerveConstants.GYRO_ID);

    estimatedPosition = new Pose2d();
    autoStartingPosition = new Translation2d();
    gyroAngle = gyro.getRotation2d().plus(Rotation2d.k180deg);
    simHeading = new Rotation2d();

    odometry =
        new SwerveDrivePoseEstimator(
            SwerveConstants.KINEMATICS, gyroAngle, this.getModulePostitions(), estimatedPosition);

    // if (isReal) {
    //   System.out.println("Isreal triggered!!!");
    //   odometryUpdater =
    //       new Notifier(
    //           () -> {
    //             modules.frontLeft.updateLogs();
    //             modules.frontRight.updateLogs();
    //             modules.backLeft.updateLogs();
    //             modules.backRight.updateLogs();

    //             estimatedPosition =
    //                 odometry.updateWithTime(
    //                     Timer.getFPGATimestamp(),
    //                     isBlue ? gyro.getRotation2d() :
    // gyro.getRotation2d().minus(Rotation2d.kPi),
    //                     getModulePostitions());
    //           });
    //   odometryUpdater.startPeriodic(0.005);
    // }

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
    estimatedPosition =
        odometry.update(
            isBlue ? gyroAngle : gyroAngle.minus(Rotation2d.kPi), getModulePostitions());

    modules.frontLeft.periodic();
    modules.frontRight.periodic();
    modules.backLeft.periodic();
    modules.backRight.periodic();

    gyroAngle = gyro.getRotation2d().plus(Rotation2d.k180deg);

    measuredStates[0] = modules.frontLeft.getModuleState();
    measuredStates[1] = modules.frontRight.getModuleState();
    measuredStates[2] = modules.backLeft.getModuleState();
    measuredStates[3] = modules.backRight.getModuleState();
  }

  @Override
  public void simulationPeriodic() {
    modules.frontLeft.updateLogs();
    modules.frontRight.updateLogs();
    modules.backLeft.updateLogs();
    modules.backRight.updateLogs();

    simHeading =
        simHeading.plus(
            new Rotation2d(
                getChassisSpeeds().omegaRadiansPerSecond * RobotConstants.CLOCK_SPEED.in(Seconds)));
    gyroAngle = simHeading;

    modules.frontLeft.simulationPeriodic();
    modules.frontRight.simulationPeriodic();
    modules.backLeft.simulationPeriodic();
    modules.backRight.simulationPeriodic();
  }

  public Pose2d getEstimatedPosition() {
    estimatedPosition = odometry.getEstimatedPosition();
    return estimatedPosition;
  }

  public void addVisionMeasurement(VisionMeasurement measurement) {
    odometry.addVisionMeasurement(
        measurement.pose,
        measurement.timestamp,
        VecBuilder.fill(
            measurement.translationStdev, measurement.translationStdev, measurement.rotationStdev));
  }

  // ===================== Gyro ===================== \\

  private void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d().plus(Rotation2d.k180deg);
  }

  public void setOdometry(Pose2d pose) {
    simHeading = pose.getRotation();
    odometry.resetPose(pose);
  }

  // ===================== Module Positions ===================== \\

  public SwerveModulePosition[] getModulePostitions() {
    return new SwerveModulePosition[] {
      modules.frontLeft.getPosition(),
      modules.frontRight.getPosition(),
      modules.backLeft.getPosition(),
      modules.backRight.getPosition()
    };
  }

  // ===================== Chassis Speeds ===================== \\

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.KINEMATICS.toChassisSpeeds(measuredStates);
  }

  // ===================== Teleop Driving ===================== \\

  private void driveFieldRelativeScaler(double x, double y, double omega) {
    double linearMagnitude = Math.pow(Math.hypot(x, y), SwerveConstants.TRANSLATION_SCALAR);

    Rotation2d direction = new Rotation2d(y, x);

    y =
        linearMagnitude
            * -direction.getCos()
            * SwerveConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond);
    x =
        linearMagnitude
            * -direction.getSin()
            * SwerveConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond);

    omega =
        Math.pow(omega, SwerveConstants.ROTATION_SCALAR)
            * SwerveConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

    driveFieldRelative(
        MetersPerSecond.of(MathUtil.applyDeadband(x, SwerveConstants.DEADBAND)),
        MetersPerSecond.of(MathUtil.applyDeadband(y, SwerveConstants.DEADBAND)),
        RadiansPerSecond.of(MathUtil.applyDeadband(-omega, SwerveConstants.DEADBAND)));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setpointStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);

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

  public void followTrajectory(SwerveSample sample) {
    Pose2d pose = getEstimatedPosition();

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

    driveFieldRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation()));
  }

  private void driveFieldRelative(ChassisSpeeds speeds) {
    driveRobotRelative(speeds);
  }

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
            driveFieldRelativeScaler(
                MathUtil.applyDeadband(x.getAsDouble(), SwerveConstants.DEADBAND),
                MathUtil.applyDeadband(y.getAsDouble(), SwerveConstants.DEADBAND),
                MathUtil.applyDeadband(omega.getAsDouble(), SwerveConstants.DEADBAND)),
        this);
  }

  public Command zeroGyroCommand() {
    return Commands.runOnce(() -> zeroGyro(), this);
  }
}
