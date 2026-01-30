package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public class Constants {
  public static class ClimberConstants {
    public static final int MOTOR_ID = 12;
    public static final Current CURRENT_LIMIT = Current.ofBaseUnits(60, Amps);
    public static final Distance DOWN_POSITION = Distance.ofBaseUnits(0, Meters);
    public static final Distance RAISED_POSITION = Distance.ofBaseUnits(5, Meters); // placeholder
    public static final LinearVelocity DOWN_VELOCITY =
        LinearVelocity.ofBaseUnits(0, MetersPerSecond);
    public static final LinearVelocity RAISED_VELOCITY =
        LinearVelocity.ofBaseUnits(0, MetersPerSecond);
    public static final LinearVelocity VELOCITY_CONSTRAINT = LinearVelocity.ofBaseUnits(5, MetersPerSecond); // placeholder
    public static final LinearAcceleration ACCELERATION_CONSTRAINT = LinearAcceleration.ofBaseUnits(10, MetersPerSecondPerSecond); // placeholder

    public static final FeedforwardGains CLIMBER_FF =
        new FeedforwardGains.Builder().kv(0.0).ks(0.0).build(); // placeholder
    public static final PIDGains CLIMBER_PID =
        new PIDGains.Builder().kp(0.0).build(); // placeholder

    public static final double GEAR_RATIO = 36.0; // placeholder
    public static final Distance SPOOL_DIAMETER = Distance.ofBaseUnits(3, Inches); // placeholder, potentially different units
    public static final AngularVelocity ROTATIONS_PER_MINUTE = RPM.of(3000);
    public static final Distance WHEEL_DIAMETER = Distance.ofBaseUnits(3, Inches); // placeholder, potentially different units
  }

  public static class RobotConstants {
    public static final Time CLOCK = Time.ofBaseUnits(20, Milliseconds);
  }
}
