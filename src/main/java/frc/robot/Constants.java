package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
    public static final PIDGains DRIVE_PID = new PIDGains.Builder().kp(0.225).build();//.225

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

    public static class RobotConstants {
    public static final double TRACK_WIDTH = edu.wpi.first.math.util.Units.inchesToMeters(23.5);
    public static final double TRACK_LENGTH = edu.wpi.first.math.util.Units.inchesToMeters(23.5);

    public static final Time ROBOT_CLOCK_SPEED = Time.ofBaseUnits(0.02, Seconds);
  }
}
