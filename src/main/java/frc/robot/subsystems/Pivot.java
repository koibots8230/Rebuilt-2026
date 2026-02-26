package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class Pivot extends SubsystemBase {

  private final SparkMax motor;
  private final SparkMaxConfig config;
  private final SparkClosedLoopController pid;
  private final TrapezoidProfile profile;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State motorSetpoint;
  private final ArmFeedforward feedforward;
  double position;
  double setpoint;
  double current;
  double voltage;

  public Pivot() {
    motor = new SparkMax(PivotConstants.MOTOR_ID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.smartCurrentLimit((int) PivotConstants.CURRENT_LIMIT.in(Amps));
    config.closedLoop.p(PivotConstants.PID.kp);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    config.absoluteEncoder.positionConversionFactor(PivotConstants.CONVERSION_FACTOR);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pid = motor.getClosedLoopController();
    profile =
        new TrapezoidProfile(
            new Constraints(
                PivotConstants.MAX_VELOCITY.in(RadiansPerSecond),
                PivotConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
    feedforward =
        new ArmFeedforward(
            PivotConstants.FEEDFORWARD.ks,
            PivotConstants.FEEDFORWARD.kg,
            PivotConstants.FEEDFORWARD.kv);
    goal = new State(PivotConstants.UP_POSITION.getRadians(), 0);
    motorSetpoint = new State(PivotConstants.UP_POSITION.getRadians(), 0);
  }

  @Override
  public void periodic() {
    motorSetpoint = profile.calculate(RobotConstants.CLOCK_SPEED.in(Seconds), motorSetpoint, goal);
    pid.setSetpoint(
        motorSetpoint.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(motorSetpoint.position, motorSetpoint.velocity));
    position = motor.getEncoder().getPosition();
    current = motor.getOutputCurrent();
    voltage = motor.getAppliedOutput() * motor.getBusVoltage();
  }

  public boolean atPosition() {
    return (position >= (setpoint - PivotConstants.MARGIN.getRadians())
        && position <= (setpoint + PivotConstants.MARGIN.getRadians()));
  }

  @Override
  public void simulationPeriodic() {
    position = motorSetpoint.position;
  }

  private void setPosition(double angle) {
    goal = new State(angle, 0);
    setpoint = angle;
  }

  public void setupLiveTuning() {
    SmartDashboard.putNumber("Intake/pidkp", PivotConstants.PID.kp);
    SmartDashboard.putNumber("Intake/feedforwardks", PivotConstants.FEEDFORWARD.ks);
    SmartDashboard.putNumber("Intake/feedforwardkg", PivotConstants.FEEDFORWARD.kg);
    SmartDashboard.putNumber("Intake/feedforwardkv", PivotConstants.FEEDFORWARD.kv);
  }

  public void updateLiveTuning() {
    config.closedLoop.p(SmartDashboard.getNumber("Intake/pidkp", PivotConstants.PID.kp));

    feedforward.setKs(SmartDashboard.getNumber("Intake/feedforwardks", PivotConstants.FEEDFORWARD.ks));
    feedforward.setKg(SmartDashboard.getNumber("Intake/feedforwardkg", PivotConstants.FEEDFORWARD.kg));
    feedforward.setKv(SmartDashboard.getNumber("Intake/feedforwardkv", PivotConstants.FEEDFORWARD.kv));
  }

  

  public Command setPositionCommand(double angle) {
    return Commands.runOnce(() -> this.setPosition(angle), this);
  }
}
