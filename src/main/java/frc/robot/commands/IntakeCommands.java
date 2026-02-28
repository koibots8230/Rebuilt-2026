package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;


public class IntakeCommands {
    public static Command autoIntake(Intake intake, Pivot pivot, Time time) {
        return Commands.sequence(
            pivot.setPositionCommand(PivotConstants.DOWN_POSITION),
            Commands.waitTime(AutoConstants.INTAKE_DELAY),
            intake.setSpeedCommand(IntakeConstants.SPEED),
            Commands.waitTime(time),
            intake.setSpeedCommand(0)
        );
    }
}
