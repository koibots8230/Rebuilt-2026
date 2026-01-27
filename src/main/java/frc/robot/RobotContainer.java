package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
  
  private final XboxController controller;
  
  private final Swerve swerve;
  private final Intake intake;

  public RobotContainer(boolean isReal) {
    controller = new XboxController(0);

    swerve = new Swerve(isReal);
    intake = new Intake();

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
      swerve.driveFieldRelativeCommand(controller::getLeftY, controller::getLeftX, controller::getRightX)
    );

    Trigger intakeButton = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    intakeButton.onTrue(intake.setSpeedCommand(-0.35));
    intakeButton.onFalse(intake.setSpeedCommand(0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
