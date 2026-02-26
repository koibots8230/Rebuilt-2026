package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
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

  public static class IndexerConstants {
    public static final double SHOOTING_SPEED = 0.5;

    public static final int MAX_MOTOR_CURRENT_AMPS = 60;

    public static final int MOTOR_ID = 14;
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
    public static final int MOTOR_ID = 12;
  }

  public static class ShooterConstants {

    public static final AngularVelocity FLYWHEEL_SPEED = RPM.of(4700);
    public static final AngularVelocity FEEDER_SPEED = RPM.of(1650);
    public static final AngularVelocity IDLE_SPEED = RPM.of(500);

    public static final PIDGains FLYWHEEL_PID = new PIDGains.Builder().kp(0.00055).build();
    public static final FeedforwardGains FLYWHEEL_FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.0019).build(); 

    public static final PIDGains FEEDER_PID = new PIDGains.Builder().kp(0.0001).build();
    public static final FeedforwardGains FEEDER_FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.00234).build();

    public static final Current FLYWHEEL_CURRENT_LIMIT = Amps.of(60);
    public static final Current FEEDER_CURRENT_LIMIT = Amps.of(80);

    public static final int FLYWHEEL_MOTOR_ID = 20;
    public static final int FEEDER_MOTOR_ID = 21;
  }

  public static class HoodConstants {
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(90);
    public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(90);

    public static final PIDGains PID = new PIDGains.Builder().kp(0.0).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().ks(0.0).kv(0.0).kg(0.0).build();

    public static final double CONVERSION_FACTOR = 0;
    public static final Current CURRENT_LIMIT = Amps.of(60);

    public static final Rotation2d UP_POSITION = Rotation2d.fromDegrees(45);
    public static final Rotation2d DOWN_POSITION = Rotation2d.fromDegrees(0);

    public static final int MOTOR_ID = 22;
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

    public static final int MOTOR_ID = 60;
  }

  public static class RobotConstants {
    public static final Time CLOCK_SPEED = Milliseconds.of(20);
  }
}
