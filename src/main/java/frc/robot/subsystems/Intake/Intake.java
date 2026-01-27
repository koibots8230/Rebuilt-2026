package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    private final SparkFlex motor;

    public Intake() {
        motor = new SparkFlex(10, MotorType.kBrushless);
    }

    private void setSpeed(double percent) {
        motor.set(percent);
    }

    public Command setSpeedCommand(double percent) {
        return Commands.runOnce(() -> this.setSpeed(percent), this);
    }
}
