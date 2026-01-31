package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final SparkMax motor;
  private final SparkMaxConfig config;
  double setpoint;
  double current;
  double voltage;
  double velocity;

  public Intake() {
    motor = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.smartCurrentLimit((int) IntakeConstants.CURRENT_LIMIT.in(Amps));
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    current = motor.getOutputCurrent();
    voltage = motor.getAppliedOutput() * motor.getBusVoltage();
    velocity = motor.getEncoder().getVelocity();
  }

  public void simulationPeriodic() {
    velocity = setpoint;
  }

  private void setSpeed(double percent) {
    motor.set(percent);
    setpoint = percent;
  }

  public Command setSpeedCommand(double percent) {
    return Commands.runOnce(() -> this.setSpeed(percent), this);
  }
}
