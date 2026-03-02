package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.LEDMode;

@Logged
public class RobotContainer {
  @NotLogged private final XboxController controller;
  private final Climber climber;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Pivot pivot;
  private final LED led;

  private final LEDMode autonomousLEDMode;
  private final LEDMode climbLEDAnimation;

  public RobotContainer() {
    climber = new Climber();
    shooter = new Shooter();
    indexer = new Indexer();
    intake = new Intake();
    pivot = new Pivot();
    led = new LED();

    controller = new XboxController(0);
    autonomousLEDMode = led.new LEDMode(1, "Autonomous");
    climbLEDAnimation = led.new LEDMode(2, "ClimbAnimation");

    configureBindings();
  }  

  private void configureBindings() {
    Trigger intakeButton = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    intakeButton.onTrue(intake.setSpeedCommand(IntakeConstants.SPEED));
    intakeButton.onFalse(intake.setSpeedCommand(0));

    Trigger pivotUp = new Trigger(controller::getAButton);
    pivotUp.onTrue(pivot.setPositionCommand(PivotConstants.UP_POSITION.getRadians()));

    Trigger pivotDown = new Trigger(controller::getBButton);
    pivotDown.onTrue(pivot.setPositionCommand(PivotConstants.DOWN_POSITION.getRadians()));

    Trigger raiseClimber = new Trigger(() -> controller.getPOV() == 0);
    raiseClimber.onTrue(
      Commands.parallel(
        climber.raiseClimbCommand(), 
        led.setModeCommand(climbLEDAnimation)));

    Trigger lowerClimber = new Trigger(() -> controller.getPOV() == 180);
    lowerClimber.onTrue(climber.lowerClimbCommand());

    Trigger shootTrigger = new Trigger(() -> controller.getRightTriggerAxis() > 0.15);
    shootTrigger.onTrue(
        Commands.parallel(
            shooter.setVelocityCommand(
                ShooterConstants.FLYWHEEL_SPEED, ShooterConstants.INTAKE_SPEED),
            indexer.setSpeedCommand(IndexerConstants.SHOOTING_SPEED)));
    shootTrigger.onFalse(
        Commands.parallel(
            shooter.setVelocityCommand(RPM.of(0), RPM.of(0)), indexer.setSpeedCommand(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.parallel(
      led.setModeCommand(autonomousLEDMode));
  }
}
