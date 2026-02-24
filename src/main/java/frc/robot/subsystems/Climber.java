package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class Climber extends SubsystemBase {
  @NotLogged private final SparkMax motor;
  @NotLogged private final SparkClosedLoopController controller;
  @NotLogged private final SparkMaxConfig config;
  @NotLogged private final RelativeEncoder encoder;
  @NotLogged private final TrapezoidProfile profile;
  @NotLogged private final SimpleMotorFeedforward feedForward;

  private Current current;
  private Voltage voltage;

  private double velocity;
  private double setpoint;
  private double position;

  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State motorSetpoint;

  public Climber() {
    motor = new SparkMax(ClimberConstants.MOTOR_ID, MotorType.kBrushless);
    config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit((int) ClimberConstants.CURRENT_LIMIT.in(Amps));
    config.inverted(true);

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    setpoint = ClimberConstants.DOWN_POSITION.in(Meters);

    position = encoder.getPosition();
    velocity = encoder.getVelocity();

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ClimberConstants.VELOCITY_CONSTRAINT.in(MetersPerSecond),
                ClimberConstants.ACCELERATION_CONSTRAINT.in(MetersPerSecondPerSecond)));
    goal = new TrapezoidProfile.State(0, 0);
    motorSetpoint = new TrapezoidProfile.State(0, 0);

    controller = motor.getClosedLoopController();

    feedForward =
        new SimpleMotorFeedforward(ClimberConstants.CLIMBER_FF.ks, ClimberConstants.CLIMBER_FF.kv);

    /*
     * 42 counts per revolution
     * 1 spool rotation = 3"
     * 1 spool rotations = 36 neo rotations
     * 1/36 spool rotations = 1 neo rotation
     * 1 neo rotation = 3/36" = 1/12"
     * 1 encoder count = 1/12" / 42
     *
     * real measurements:
     * top: 21.5 cm
     * bottom: 17.5 cm
     * delta: 4cm
     * advantage scope measurement: 4cm too
     *
     * velocity:
     * 1.4 cm/s up in advantage scope
     * real life: 4s so 1 cm/s
     *
     * kV of 60 = 1.2 V, 1.8 cm/s
     *
     */
    double realConversionFactor = ((1.0 / 12)) / 39.37;

    config.encoder.positionConversionFactor(realConversionFactor);
    config.encoder.velocityConversionFactor(realConversionFactor / 60);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // config.encoder.positionConversionFactor(
    //     (Math.PI * ClimberConstants.SPOOL_DIAMETER.in(Inches)) / ClimberConstants.GEAR_RATIO);
    // config.encoder.velocityConversionFactor(
    //     (ClimberConstants.ROTATIONS_PER_MINUTE.in(RPM) *
    // ClimberConstants.WHEEL_DIAMETER.in(Inches))
    //         / (ClimberConstants.GEAR_RATIO * 60));
  }

  @Override
  public void periodic() {
    motorSetpoint = profile.calculate(RobotConstants.CLOCK_SPEED.in(Seconds), motorSetpoint, goal);

    controller.setSetpoint(
        motorSetpoint.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedForward.calculate((motorSetpoint.velocity)));

    position = encoder.getPosition();
    velocity = encoder.getVelocity();

    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    setpoint = goal.position;
  }

  @Override
  public void simulationPeriodic() {
    motorSetpoint = profile.calculate(RobotConstants.CLOCK_SPEED.in(Seconds), motorSetpoint, goal);
  }

  private void setGoal(double position, LinearVelocity velocity) {
    goal = new TrapezoidProfile.State(position, velocity.in(MetersPerSecond));
  }

  private void zeroEncoder() {
    encoder.setPosition(0);
  }

  private void setSpeed(double speed) {
    motor.set(speed);
  }

  public Command raiseClimbCommand() {
    return Commands.runOnce(
        () ->
            this.setGoal(
                ClimberConstants.RAISED_POSITION.in(Meters), ClimberConstants.RAISED_VELOCITY),
        this);
  }

  public Command lowerClimbCommand() {
    return Commands.runOnce(
        () ->
            this.setGoal(ClimberConstants.DOWN_POSITION.in(Meters), ClimberConstants.DOWN_VELOCITY),
        this);
  }

  public Command lowerClimbManualCommand(double speed) {
    return Commands.sequence(
        Commands.run(() -> setSpeed(speed), this), Commands.run(() -> zeroEncoder(), this));
  }
}
