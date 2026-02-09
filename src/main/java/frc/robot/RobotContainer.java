package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
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
  private final Swerve swerve;

  private AngularVelocity shooterShoot;
  private AngularVelocity shooterIntake;
  private Rotation2d pivotUpPosition;
  private Rotation2d pivotDownPosition;

  public RobotContainer(boolean isReal) {
    climber = new Climber();
    shooter = new Shooter();
    indexer = new Indexer();
    intake = new Intake();
    pivot = new Pivot();
    swerve = new Swerve(isReal);

    controller = new XboxController(0);

    shooterShoot = ShooterConstants.FLYWHEEL_SPEED;
    shooterIntake = ShooterConstants.INTAKE_SPEED;
    pivotUpPosition = PivotConstants.UP_POSITION;
    pivotDownPosition = PivotConstants.DOWN_POSITION;

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCommand(
            controller::getLeftY, controller::getLeftX, controller::getRightX));

    Trigger intakeButton = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    intakeButton.onTrue(intake.setSpeedCommand(IntakeConstants.SPEED));
    intakeButton.onFalse(intake.setSpeedCommand(0));

    Trigger pivotUp = new Trigger(controller::getAButton);
    pivotUp.onTrue(pivot.setPositionCommand(pivotUpPosition.getRadians()));

    Trigger pivotDown = new Trigger(controller::getBButton);
    pivotDown.onTrue(pivot.setPositionCommand(pivotDownPosition.getRadians()));

    Trigger raiseClimber = new Trigger(() -> controller.getPOV() == 0);
    raiseClimber.onTrue(climber.raiseClimbCommand());

    Trigger lowerClimber = new Trigger(() -> controller.getPOV() == 180);
    lowerClimber.onTrue(climber.lowerClimbCommand());

    Trigger shootTrigger = new Trigger(() -> controller.getRightTriggerAxis() > 0.15);
    shootTrigger.onTrue(
        shooter.setVelocityCommand(shooterShoot, shooterIntake));
    shootTrigger.onFalse(shooter.setVelocityCommand(RPM.of(0), RPM.of(0)));
  }

  public void setupLiveTuning() {
    SmartDashboard.putNumber("Shooter/shootSetPoint", ShooterConstants.FLYWHEEL_SPEED.in(RPM));
    SmartDashboard.putNumber("Shooter/intakeSetPoint", ShooterConstants.INTAKE_SPEED.in(RPM));
    SmartDashboard.putNumber("Pivot/pivotUpPoint", PivotConstants.UP_POSITION.getDegrees());
    SmartDashboard.putNumber("Pivot/pivotDownPoint", PivotConstants.DOWN_POSITION.getDegrees());
    shooter.setupLiveTuning();
    pivot.setupLiveTuning();
  }

  public void updateLiveTuning() {
    shooterShoot = RPM.of(SmartDashboard.getNumber("Shooter/shootSetPoint", ShooterConstants.FLYWHEEL_SPEED.in(RPM)));
    shooterIntake = RPM.of(SmartDashboard.getNumber("Shooter/intakeSetPoint", ShooterConstants.INTAKE_SPEED.in(RPM)));
    pivotUpPosition = Rotation2d.fromDegrees(SmartDashboard.getNumber("Pivot/pivotUpPoint", PivotConstants.UP_POSITION.getDegrees()));
    pivotDownPosition = Rotation2d.fromDegrees(SmartDashboard.getNumber("Pivot/pivotDownPoint", PivotConstants.DOWN_POSITION.getDegrees()));
    shooter.updateLiveTuning();
    pivot.setupLiveTuning();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
