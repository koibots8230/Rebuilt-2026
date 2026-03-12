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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
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
  private final ShooterHood shooterHood;
  private double hoodAngle;
  private Distance distanceToHub;
  private final Autos autos;
  private final Vision vision;

  private double shooterVelocity;
  private double hoodPosition;

  public RobotContainer(boolean isReal) {
    climber = new Climber();
    intake = new Intake();
    shooter = new Shooter();
    indexer = new Indexer();
    pivot = new Pivot();
    shooterHood = new ShooterHood();
    swerve = new Swerve(isReal);
    vision =
        new Vision(
            swerve::getEstimatedPosition,
            swerve::getGyroAngle,
            swerve::addVisionMeasurement,
            swerve::getIsBlue);

    controller = new XboxController(0);

    shooterVelocity = ShooterConstants.FLYWHEEL_SPEED.in(RPM);
    hoodPosition = HoodConstants.DOWN_POSITION.getDegrees();
    autos = new Autos(swerve, shooter, indexer, intake, pivot, climber);
    hoodAngle = HoodConstants.DOWN_POSITION.getDegrees();
    //distanceToHub = Meters.of(0.0); // This will eventually be set by vision
    

    configureBindings();
  }

  public void setIsBlue() {
    swerve.setIsBlue(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    shooterHood.setIsBlue(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCommand(
            controller::getLeftY, controller::getLeftX, controller::getRightX));

    Trigger intakeButton = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    intakeButton.onTrue(intake.setSpeedCommand(IntakeConstants.SPEED));
    intakeButton.onFalse(intake.setSpeedCommand(0));

    Trigger intakeReverse = new Trigger(controller::getLeftBumperButton);
    intakeReverse.onTrue(
        Commands.parallel(
            indexer.setSpeedCommand(-IndexerConstants.SHOOTING_SPEED),
            intake.setSpeedCommand(-IntakeConstants.SPEED)));
    intakeReverse.onFalse(Commands.parallel(indexer.setSpeedCommand(0), intake.setSpeedCommand(0)));

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
    shootTrigger.whileTrue(
        Commands.sequence(
            shooter.setVelocityCommand(ShooterConstants.FEEDER_SPEED, RPM.of(shooterVelocity)),
            Commands.waitUntil(shooter::atSpeed),
            indexer.setSpeedCommand(IndexerConstants.SHOOTING_SPEED),
            intake.setSpeedCommand(IntakeConstants.SPEED),
            Commands.sequence(
                    pivot.setPositionCommand(PivotConstants.MID_POSITION),
                    Commands.waitUntil(pivot::atPosition),
                    pivot.setPositionCommand(PivotConstants.DOWN_POSITION),
                    Commands.waitUntil(pivot::atPosition))
                .repeatedly()));
    shootTrigger.onFalse(
        Commands.parallel(
            shooter.setVelocityCommand(ShooterConstants.IDLE_SPEED, RPM.of(0))));
    Trigger hoodTrigger = new Trigger(() -> controller.getLeftBumperButton());
    hoodTrigger.onTrue(shooterHood.setPositionCommand(hoodAngle));
    hoodTrigger.onFalse(shooterHood.setPositionCommand(HoodConstants.DOWN_POSITION.getRadians()));

    Trigger zeroGyro = new Trigger(() -> controller.getAButton() && controller.getYButton());
    zeroGyro.onTrue(swerve.zeroGyroCommand());
  }

  public void setupLiveTuning() {
    shooter.setupLiveTuning();
    shooterHood.setupLiveTuning();
    pivot.setupLiveTuning();

    SmartDashboard.putNumber("flywheelVelocity", shooterVelocity);
    SmartDashboard.putNumber("hoodPosition", hoodPosition);
  }

  public void updateLiveTuning() {
    shooterVelocity =
        SmartDashboard.getNumber("flywheelVelocity", ShooterConstants.FLYWHEEL_SPEED.in(RPM));
    hoodPosition = SmartDashboard.getNumber("hoodPosition", HoodConstants.DOWN_POSITION.getDegrees());
    shooterHood.setPositionCommand(hoodPosition);
    shooter.updateLiveTuning();
    pivot.updateLiveTuning();
  }
}
