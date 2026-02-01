package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final SparkMax motor;

  public Indexer() {
    motor = new SparkMax(10, MotorType.kBrushless);
  }

  private void setSpeed(double percent) {
    motor.set(percent);
  }

  public Command setSpeedCommand(double percent) {
    return Commands.runOnce(() -> this.setSpeed(percent), this);
  }
}
