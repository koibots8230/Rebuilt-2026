package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Pivot;

@Logged
public class RobotContainer {
  
  private final XboxController controller;
  
  private final Swerve swerve;
  private final Intake intake;
  private final Pivot pivot;

  public RobotContainer(boolean isReal) {
    controller = new XboxController(0);

    swerve = new Swerve(isReal);
    intake = new Intake();
    pivot = new Pivot();

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
      swerve.driveFieldRelativeCommand(controller::getLeftY, controller::getLeftX, controller::getRightX)
    );

    final Trigger intakeButton = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    intakeButton.onTrue(intake.setSpeedCommand(-0.35));
    intakeButton.onFalse(intake.setSpeedCommand(0));

    Trigger pivotUp = new Trigger(() -> controller.getLeftBumperButtonPressed() == true);
    pivotUp.onTrue(pivot.setSpeedCommand(0.15));

    Trigger pivotDown = new Trigger(() -> controller.getRightBumperButtonPressed() == true);
    pivotDown.onTrue(pivot.setSpeedCommand(-0.15));

    Trigger pivotOff = new Trigger(() -> controller.getLeftBumperButtonPressed() == false  && controller.getRightBumperButtonPressed() == false);
    pivotOff.onTrue(pivot.setSpeedCommand(0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
