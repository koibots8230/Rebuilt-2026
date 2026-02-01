package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Indexer extends SubsystemBase {
  private final SparkMax motor;

  public Indexer() {
    motor = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);

    // motor.resetToFactory() how do I make sure that the motor are in the state
    // that we want
  }

  private void setSpeed(double percent) {
    motor.set(percent);

    SmartDashboard.putString("INDEXER CURRENT SPEED", Double.toString(percent));
  }

  public Command setSpeedCommand(double percent) {
    return Commands.runOnce(() -> this.setSpeed(percent), this);
  }
}
