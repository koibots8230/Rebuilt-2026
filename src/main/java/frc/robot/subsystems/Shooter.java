package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  @NotLogged private final SparkFlex motor;
  @NotLogged private final SparkFlexConfig motorConfig;
  @NotLogged private final SparkClosedLoopController motorController;
  private Voltage voltage;
  private AngularVelocity velocity;
  private Current current;
  private AngularVelocity setpoint;

  public Shooter() {
    motor = new SparkFlex(ShooterConstants.MOTOR_PORT, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();
    motorConfig.closedLoop.p(ShooterConstants.PID.kp);
    motorConfig.closedLoop.feedForward.kV(ShooterConstants.FEEDFORWARD.kv);

    motorConfig.smartCurrentLimit((int) ShooterConstants.CURRENT_LIMIT.in(Amps));
    motorConfig.inverted(true);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // I don't know why this is angry, but it is.
    motorController = motor.getClosedLoopController();

    voltage = Volts.of(0);
    velocity = RPM.of(0);
    current = Amps.of(0);
    setpoint = RPM.of(0);
  }

  @Override
  public void periodic() {
    voltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    velocity = RPM.of(motor.getEncoder().getVelocity());
    current = Amps.of(motor.getOutputCurrent());
  }

  private void setVelocity(AngularVelocity velocity) {
    motorController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    // Deprecated
    setpoint = velocity;
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return Commands.runOnce(() -> setVelocity(velocity), this);
  }
}
