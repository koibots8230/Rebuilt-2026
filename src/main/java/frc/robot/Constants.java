package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public class Constants {
  public static class ClimberConstants {
    public static final int MOTOR_ID = 12;
    public static final Current CURRENT_LIMIT = Current.ofBaseUnits(60, Amps);
    public static final double DOWN_POSITION = 0;
    public static final double RAISED_POSITION = 0; // placeholder
    public static final LinearVelocity DOWN_VELOCITY = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
    public static final LinearVelocity RAISED_VELOCITY = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
    public static final double VELOCITY_CONSTRAINT = 5; // placeholder
    public static final double ACCELERATION_CONSTRAINT = 10; // placeholder

    public static final FeedforwardGains CLIMBER_FF = new FeedforwardGains.Builder().kv(0.0).ks(0.0).build(); // placeholder
    public static final PIDGains CLIMBER_PID = new PIDGains.Builder().kp(0.0).build();  // placeholder
  }
  
  public static class RobotConstants {
    public static final double CLOCK = 0.02;
  }
}
