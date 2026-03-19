package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.LEDMode;

@Logged
public class RobotContainer {
  @NotLogged private final XboxController controller;
  private final Climber climber;
  private final Swerve swerve;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Pivot pivot;
  private final LED led;

  private final LEDMode autonomousLEDMode;
  private final LEDMode climbLEDAnimation;
  private final Autos autos;
  private final Vision vision;

  private double shooterVelocity;

  public RobotContainer(boolean isReal) {
    climber = new Climber();
    intake = new Intake();
    shooter = new Shooter();
    indexer = new Indexer();
    pivot = new Pivot();
    swerve = new Swerve(isReal);
    vision =
        new Vision(
            swerve::getEstimatedPosition,
            swerve::getGyroAngle,
            swerve::addVisionMeasurement,
            swerve::getIsBlue);
    led = new LED();

    controller = new XboxController(0);

    shooterVelocity = ShooterConstants.FLYWHEEL_SPEED.in(RPM);

    autos = new Autos(swerve, shooter, indexer, intake, pivot, climber);
    autonomousLEDMode = led.new LEDMode(1, "Autonomous");
    climbLEDAnimation = led.new LEDMode(5, "ClimbAnimation");

    configureBindings();
  }  

  public void setIsBlue() {
    swerve.setIsBlue(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
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
    //Arm is going up, but robot is not.

    Trigger lowerClimber = new Trigger(() -> controller.getPOV() == 180);
    lowerClimber.onTrue(
      commands.parallel(
        climber.lowerClimbCommand(),
        led.setModeCommand(climbLEDAnimation)));
    //Arm is going down, robot is going up.

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
            shooter.setVelocityCommand(ShooterConstants.IDLE_SPEED, RPM.of(0)),
            indexer.setSpeedCommand(0),
            intake.setSpeedCommand(0),
            pivot.setPositionCommand(PivotConstants.DOWN_POSITION)));

    Trigger zeroGyro = new Trigger(() -> controller.getAButton() && controller.getYButton());
    zeroGyro.onTrue(swerve.zeroGyroCommand());

    Trigger configureLEDTrigger = new Trigger(() -> DriverStation.getGameSpecificMessage().length() > 0);
    configureLEDTrigger.onTrue(led.configureLogicCommand());
  }

  public void setupLiveTuning() {
    shooter.setupLiveTuning();
    pivot.setupLiveTuning();

    SmartDashboard.putNumber("flywheelVelocity", shooterVelocity);
  }

  public void updateLiveTuning() {
    shooterVelocity =
        SmartDashboard.getNumber("flywheelVelocity", ShooterConstants.FLYWHEEL_SPEED.in(RPM));
    shooter.updateLiveTuning();
    pivot.updateLiveTuning();
            shooter.setVelocityCommand(RPM.of(0), RPM.of(0)), indexer.setSpeedCommand(0)));

    
  }

  public Command getAutonomousCommand() {
    return Commands.parallel(
      led.setModeCommand(autonomousLEDMode));
  }
}
