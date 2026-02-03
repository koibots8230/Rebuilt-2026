package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

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

    public static final LinearVelocity MAX_LINEAR_VELOCITY = LinearVelocity.ofBaseUnits(.2, Units.MetersPerSecond);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = AngularVelocity.ofBaseUnits(Math.PI * 2, Units.RadiansPerSecond);

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

    public static final Current DRIVE_CURRENT_LIMIT = Units.Amps.of(80);
    public static final Current TURN_CURRENT_LIMIT = Units.Amps.of(30);

  }

  public static class RobotConstants {
    public static final Distance ROBOT_WIDTH = Units.Meters.of(0.6985); // 27.5 inches
    public static final Distance ROBOT_LENGTH = Units.Meters.of(0.6985);
    public static final double CLOCK = 50;

  }
}