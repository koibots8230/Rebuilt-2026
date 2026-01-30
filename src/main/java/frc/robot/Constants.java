package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public class Constants {
    public static class ShooterConstants {

        public static final AngularVelocity SHOOT_SPEED = RPM.of(3000);
        public static final PIDGains PID = new PIDGains.Builder().kp(0.0).build();
        public static final FeedforwardGains FEEDFORWARD =
            new FeedforwardGains.Builder().kv(0.0).build();
        public static final int MOTOR_PORT = 0;
        public static final Current CURRENT_LIMIT = Amps.of(40);

    }
}
