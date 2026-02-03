package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public class Constants {

  public static class IndexerConstants {
    public static final int MOTOR_ID = 10;
    public static final int MAX_MOTOR_CURRENT_AMPS = 60;
  }

  public static class IntakeConstants {
    public static final double SPEED = 0.35;
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
    public static final int MOTOR_ID = 11;
  }

  public static class RobotConstants {
    public static final Time CLOCK_SPEED = Milliseconds.of(20);
  }
}
