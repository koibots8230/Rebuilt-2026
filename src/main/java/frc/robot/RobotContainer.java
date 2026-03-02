package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  @NotLogged private final XboxController controller;
  private final Climber climber;
  private final Swerve swerve;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Pivot pivot;
  private final Autos autos;
  private final Vision vision;

  public RobotContainer(boolean isReal) {
    climber = new Climber();
    intake = new Intake();
    shooter = new Shooter();
    indexer = new Indexer();
    pivot = new Pivot();
    swerve = new Swerve(isReal);
    swerve.setIsBlue(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    vision = new Vision(swerve::getEstimatedPosition, swerve::getGyroAngle, swerve::addVisionMeasurement, swerve::getIsBlue);

    controller = new XboxController(0);

    autos = new Autos(swerve, shooter, indexer, intake, pivot, climber);

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCommand(
            controller::getLeftY, controller::getLeftX, controller::getRightX));

    // Trigger intakeButton = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    // intakeButton.onTrue(intake.setSpeedCommand(IntakeConstants.SPEED));
    // intakeButton.onFalse(intake.setSpeedCommand(0));

    Trigger pivotUp = new Trigger(controller::getAButton);
    pivotUp.onTrue(pivot.setPositionCommand(PivotConstants.UP_POSITION));

    Trigger pivotDown = new Trigger(controller::getBButton);
    pivotDown.onTrue(pivot.setPositionCommand(PivotConstants.DOWN_POSITION));

    Trigger raiseClimber = new Trigger(() -> controller.getPOV() == 0);
    raiseClimber.onTrue(climber.raiseClimbCommand());

    Trigger lowerClimber = new Trigger(() -> controller.getPOV() == 180);
    lowerClimber.onTrue(climber.lowerClimbCommand());

    Trigger zeroClimber = new Trigger(() -> controller.getYButton());
    zeroClimber.onTrue(climber.lowerClimbManualCommand(ClimberConstants.MANUAL_LOWER_SPEED));
    zeroClimber.onFalse(climber.lowerClimbManualCommand(0.0));

    Trigger shootTrigger = new Trigger(() -> controller.getRightTriggerAxis() > 0.15);
    shootTrigger.onTrue(ShootCommands.shoot(shooter, indexer));
    shootTrigger.onFalse(ShootCommands.stop(shooter, indexer));

    swerve.setDefaultCommand(
        swerve.driveCommand(controller::getLeftY, controller::getLeftX, controller::getRightX));
  }

  public void setupLiveTuning() {
    pivot.setupLiveTuning();
  }

  public void updateLiveTuning() {
    pivot.setupLiveTuning();
  }
}
