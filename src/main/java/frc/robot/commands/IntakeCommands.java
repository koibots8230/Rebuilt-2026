package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;


public class IntakeCommands {
    public static Command autoIntake(Intake intake, Time time) {
        return Commands.sequence(
            intake.setSpeedCommand(IntakeConstants.SPEED),
            Commands.waitTime(time),
            intake.setSpeedCommand(0)
        );
    }
}
