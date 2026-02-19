package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final ShooterHood shooterHood;
  private double hoodAngle;
  private Distance distanceToHub;

  public RobotContainer() {
    climber = new Climber();
    shooter = new Shooter();
    indexer = new Indexer();
    intake = new Intake();
    pivot = new Pivot();
    shooterHood = new ShooterHood();

    controller = new XboxController(0);
    hoodAngle = HoodConstants.DOWN_POSITION.getDegrees();
    distanceToHub = Meters.of(0.0); // This will eventually be set by vision
    

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
    raiseClimber.onTrue(climber.raiseClimbCommand());

    Trigger lowerClimber = new Trigger(() -> controller.getPOV() == 180);
    lowerClimber.onTrue(climber.lowerClimbCommand());

    Trigger shootTrigger = new Trigger(() -> controller.getRightTriggerAxis() > 0.15);
    shootTrigger.onTrue(
        Commands.parallel(
            shooter.setVelocityCommand(
                ShooterConstants.FLYWHEEL_SPEED, ShooterConstants.FEEDER_SPEED),
            indexer.setSpeedCommand(IndexerConstants.SHOOTING_SPEED)));
    shootTrigger.onFalse(
        Commands.parallel(
            shooter.setVelocityCommand(RPM.of(500), RPM.of(500)), indexer.setSpeedCommand(0)));
    Trigger hoodTrigger = new Trigger(() -> controller.getLeftBumper());
    hoodTrigger.onTrue(shooterHood.setPositionCommand(hoodAngle));
    hoodTrigger.onFalse(shooterHood.setPositionCommand(HoodConstants.DOWN_POSITION.getRadians()));

    Trigger autoShootTrigger = new Trigger(() -> controller.getRightBumper());
    autoShootTrigger.onTrue(
        shooterHood.autoSetPositionCommand(distanceToHub));
    
  }

  public void setupLiveTuning() {
    SmartDashboard.putNumber("Shooter Hood Angle", HoodConstants.DOWN_POSITION.getDegrees());
  }

  public void applyLiveTuning() {
    hoodAngle = (SmartDashboard.getNumber("Shooter Hood Angle", HoodConstants.DOWN_POSITION.getDegrees()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
