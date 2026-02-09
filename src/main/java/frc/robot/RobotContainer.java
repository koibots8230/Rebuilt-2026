package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  @NotLogged private final XboxController controller;
  private final Climber climber;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Pivot pivot;
  private final Swerve swerve;

  public RobotContainer(boolean isReal) {
    climber = new Climber();
    shooter = new Shooter();
    indexer = new Indexer();
    intake = new Intake();
    pivot = new Pivot();
    swerve = new Swerve(isReal);

    controller = new XboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCommand(
            controller::getLeftY, controller::getLeftX, controller::getRightX));

    Trigger intakeButton = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    intakeButton.onTrue(intake.setSpeedCommand(IntakeConstants.SPEED));
    intakeButton.onFalse(intake.setSpeedCommand(0));

    // Trigger pivotUp = new Trigger(controller::getAButton);
    // pivotUp.onTrue(pivot.setPositionCommand(PivotConstants.UP_POSITION.getRadians()));

    Trigger pivotDown = new Trigger(controller::getBButton);
    pivotDown.onTrue(pivot.setPositionCommand(PivotConstants.DOWN_POSITION.getRadians()));

    Trigger raiseClimber = new Trigger(() -> controller.getPOV() == 0);
    raiseClimber.onTrue(climber.raiseClimbCommand());

    Trigger lowerClimber = new Trigger(() -> controller.getPOV() == 180);
    lowerClimber.onTrue(climber.lowerClimbCommand());

    Trigger shootTrigger = new Trigger(controller::getAButton);
    shootTrigger.onTrue(
        Commands.parallel(
          shooter.setVelocityCommand(ShooterConstants.FLYWHEEL_SPEED, ShooterConstants.INTAKE_SPEED),
          Commands.sequence(
            pivot.setPositionCommand(PivotConstants.MID_POSITION.getRadians()),
            Commands.waitUntil(pivot::atPosition),
            pivot.setPositionCommand(PivotConstants.DOWN_POSITION.getRadians()),
            Commands.waitUntil(pivot::atPosition)
          ).repeatedly()
    ));
    shootTrigger.onFalse(
      Commands.parallel(
        shooter.setVelocityCommand(RPM.of(0), RPM.of(0)),
        pivot.setPositionCommand(PivotConstants.DOWN_POSITION.getRadians())
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
