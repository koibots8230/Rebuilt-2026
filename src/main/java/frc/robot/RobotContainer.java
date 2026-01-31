package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Pivot;

@Logged
public class RobotContainer {

  private final XboxController controller;

  private final Intake intake;
  private final Pivot pivot;

  public RobotContainer(boolean isReal) {
    controller = new XboxController(0);

    intake = new Intake();
    pivot = new Pivot();

    configureBindings();
  }

  private void configureBindings() {
    final Trigger intakeButton = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    intakeButton.onTrue(intake.setSpeedCommand(IntakeConstants.SPEED));
    intakeButton.onFalse(intake.setSpeedCommand(0));

    Trigger pivotUp = new Trigger(controller::getAButton);
    pivotUp.onTrue(pivot.setPositionCommand(PivotConstants.UP_POSITION.getRadians()));

    Trigger pivotDown = new Trigger(controller::getBButton);
    pivotDown.onTrue(pivot.setPositionCommand(PivotConstants.DOWN_POSITION.getRadians()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
