package frc.robot;

import edu.wpi.first.math.util.Units.*;

public class Constants {

  public static class SwerveConstants {


    // These are placeholder IDs; replace with actual motor IDs
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_LEFT_TURN_MOTOR_ID = 2;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 4;
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
    public static final int BACK_LEFT_TURN_MOTOR_ID = 6;
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
    public static final int BACK_RIGHT_TURN_MOTOR_ID = 8;

    public static final int GYRO_ID = 9;

    public static final LinearVelocity MAX_LINEAR_VELOCITY = LinearVelocity.ofBaseUnits(.2, MetersPerSecond);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = AngularVelocity.ofBaseUnits(Math.PI * 2, RadiansPerSecond);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(RobotConstants.ROBOT_LENGTH / 2, RobotConstants.ROBOT_WIDTH / 2),
      new Translation2d(RobotConstants.ROBOT_LENGTH / 2, -RobotConstants.ROBOT_WIDTH / 2),
      new Translation2d(-RobotConstants.ROBOT_LENGTH / 2, RobotConstants.ROBOT_WIDTH / 2),
      new Translation2d(-RobotConstants.ROBOT_WIDTH / 2, -RobotConstants.ROBOT_LENGTH / 2)
    );

    // PID values copped from temp drivetrain; tuneing required
    public static final PIDGains DRIVE_PID = new PIDGains.Builder().kp(0.225).build();
    public static final FeedforwardGains DRIVE_FEEDFORWARD = new FeedforwardGains.Builder().kv(2.3).build();
  
    public static final PIDGains TURN_PID = new PIDGains.Builder().kp(0.4).kd(0).build();
    public static final FeedforwardGains TURN_FEEDFORWARD = new FeedforwardGains.Builder().kv(0.45).build();

    public static final Current DRIVE_CURRENT_LIMIT = Amps.of(80);
    public static final Current TURN_CURRENT_LIMIT = Amps.of(30);

  }

  public static class RobotConstants {
    public static final Meters ROBOT_WIDTH = Meters.ofBaseUnits(Units.inchesToMeters(27.5));
    public static final Meters ROBOT_LENGTH = Meters.ofBaseUnits(Units.inchesToMeters(27.5));
    public static final double CLOCK = 50;

  }
}