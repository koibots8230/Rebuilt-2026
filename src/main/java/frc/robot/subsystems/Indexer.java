package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

@Logged
public class Indexer extends SubsystemBase {

  @NotLogged private final SparkMax motor;

  private Voltage voltage;
  private AngularVelocity velocity;
  private Current current;

  public Indexer() {
    var motorConfig =
        new SparkMaxConfig()
            .inverted(false)
            .smartCurrentLimit(IndexerConstants.MAX_MOTOR_CURRENT_AMPS)
            .idleMode(IdleMode.kBrake);

    this.motor = new SparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);

    this.motor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialising current state
    readValues();
  }

  @Override
  public void periodic() {
    readValues();
  }

  public void readValues() {
    this.current = Amps.of(this.motor.getOutputCurrent());
    this.voltage = Volts.of(this.motor.getAppliedOutput());
    this.velocity = RPM.of(this.motor.getEncoder().getVelocity());
  }

  private void setSpeed(double speedPercent) {
    this.motor.set(speedPercent);
  }

  public Command setSpeedCommand(double percent) {
    return Commands.runOnce(() -> this.setSpeed(percent), this);
  }
}
