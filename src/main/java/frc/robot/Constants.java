package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public class Constants {

  public static class SwerveConstants {
    public static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(4.25);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(2 * Math.PI);

    public static final AngularVelocity MAX_TURN_VELOCITY = RadiansPerSecond.of(60 * Math.PI);
    public static final AngularAcceleration MAX_TURN_ACCELRATION =
        RadiansPerSecondPerSecond.of(80 * Math.PI);

    public static final PIDGains TURN_PID = new PIDGains.Builder().kp(0.4).kd(0).build();
    public static final PIDGains DRIVE_PID = new PIDGains.Builder().kp(0.225).build(); // .225

    public static final FeedforwardGains TURN_FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.45).build();
    public static final FeedforwardGains DRIVE_FEEDFORWARD =
        new FeedforwardGains.Builder().kv(2.3).build();

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(RobotConstants.TRACK_LENGTH / 2.0, RobotConstants.TRACK_WIDTH / 2.0),
            new Translation2d(RobotConstants.TRACK_LENGTH / 2.0, -RobotConstants.TRACK_WIDTH / 2.0),
            new Translation2d(-RobotConstants.TRACK_LENGTH / 2.0, RobotConstants.TRACK_WIDTH / 2.0),
            new Translation2d(
                -RobotConstants.TRACK_LENGTH / 2.0, -RobotConstants.TRACK_WIDTH / 2.0));

    public static final double SWERVE_GEARING = 5.50;

    public static final double DRIVE_CONVERSION_FACTOR =
        (edu.wpi.first.math.util.Units.inchesToMeters(1.5) * 2 * Math.PI) / SWERVE_GEARING;
    public static final double TURN_CONVERSION_FACTOR = 2 * Math.PI;

    public static final Rotation2d[] OFFSETS = {
      Rotation2d.fromRadians((3 * Math.PI) / 2.0),
      Rotation2d.fromRadians(0),
      Rotation2d.fromRadians(Math.PI),
      Rotation2d.fromRadians(Math.PI / 2.0)
    };

    public static final Current TURN_CURRENT_LIMIT = Amps.of(30);
    public static final Current DRIVE_CURRENT_LIMIT = Amps.of(80);

    public static final double DEADBAND = 0.07;

    public static final double TRANSLATION_SCALAR = 2;

    public static final double ROTATION_SCALAR = 1;

    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_TURN_ID = 2;
    public static final int FRONT_RIGHT_DRIVE_ID = 3;
    public static final int FRONT_RIGHT_TURN_ID = 4;
    public static final int BACK_LEFT_DRIVE_ID = 5;
    public static final int BACK_LEFT_TURN_ID = 6;
    public static final int BACK_RIGHT_DRIVE_ID = 7;
    public static final int BACK_RIGHT_TURN_ID = 8;

    public static final int GYRO_ID = 9;
  }

  public static class IndexerConstants {
    public static final double SHOOTING_SPEED = 0.3;

    public static final int MAX_MOTOR_CURRENT_AMPS = 60;

    public static final int MOTOR_ID = 40;
  }

  public static class IntakeConstants {
    public static final double SPEED = 0.95;
    public static final Current CURRENT_LIMIT = Amps.of(60);
    public static final int MOTOR_ID = 10;
  }

  public static class PivotConstants {
    public static final Rotation2d UP_POSITION = Rotation2d.fromDegrees(90);
    public static final Rotation2d DOWN_POSITION = Rotation2d.fromDegrees(0);
    public static final PIDGains PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0).kg(0).build();
    public static final double CONVERSION_FACTOR = Math.PI * 2;
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(90);
    public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(90);
    public static final Current CURRENT_LIMIT = Amps.of(60);
    public static final int MOTOR_ID = 60;
  }

  public static class ShooterConstants {

    public static final AngularVelocity FLYWHEEL_SPEED = RPM.of(5040);
    public static final AngularVelocity INTAKE_SPEED = RPM.of(1650);

    public static final PIDGains FLYWHEEL_PID = new PIDGains.Builder().kp(0.0).build();
    public static final FeedforwardGains FLYWHEEL_FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.5).build();

    public static final PIDGains INTAKE_PID = new PIDGains.Builder().kp(0.0).build();
    public static final FeedforwardGains INTAKE_FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.5).build();

    public static final Current FLYWHEEL_CURRENT_LIMIT = Amps.of(60);
    public static final Current INTAKE_CURRENT_LIMIT = Amps.of(80);

    public static final int FLYWHEEL_MOTOR_ID = 20;
    public static final int INTAKE_MOTOR_ID = 21;
  }

  public static class ClimberConstants {
    public static final Distance DOWN_POSITION = Distance.ofBaseUnits(0, Meters);
    public static final Distance RAISED_POSITION = Distance.ofBaseUnits(5, Meters); // placeholder
    public static final LinearVelocity DOWN_VELOCITY =
        LinearVelocity.ofBaseUnits(0, MetersPerSecond);
    public static final LinearVelocity RAISED_VELOCITY =
        LinearVelocity.ofBaseUnits(0, MetersPerSecond);
    public static final LinearVelocity VELOCITY_CONSTRAINT =
        LinearVelocity.ofBaseUnits(5, MetersPerSecond); // placeholder
    public static final LinearAcceleration ACCELERATION_CONSTRAINT =
        LinearAcceleration.ofBaseUnits(10, MetersPerSecondPerSecond); // placeholder

    public static final FeedforwardGains CLIMBER_FF =
        new FeedforwardGains.Builder().kv(0.0).ks(0.0).build(); // placeholder
    public static final PIDGains CLIMBER_PID =
        new PIDGains.Builder().kp(0.0).build(); // placeholder

    public static final Current CURRENT_LIMIT = Current.ofBaseUnits(60, Amps);
    public static final AngularVelocity ROTATIONS_PER_MINUTE = RPM.of(3000);

    public static final double GEAR_RATIO = 36.0; // placeholder
    public static final Distance SPOOL_DIAMETER =
        Distance.ofBaseUnits(3, Inches); // placeholder, potentially different units
    public static final Distance WHEEL_DIAMETER =
        Distance.ofBaseUnits(3, Inches); // placeholder, potentially different units

    public static final int MOTOR_ID = 12;
  }

  public static class RobotConstants {
    public static final double TRACK_WIDTH = edu.wpi.first.math.util.Units.inchesToMeters(23.5);
    public static final double TRACK_LENGTH = edu.wpi.first.math.util.Units.inchesToMeters(23.5);
    public static final Time CLOCK_SPEED = Milliseconds.of(20);
  }
}
