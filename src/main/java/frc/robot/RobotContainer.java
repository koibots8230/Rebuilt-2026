package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
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
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;

  public RobotContainer() {
    climber = new Climber();
    shooter = new Shooter();
    indexer = new Indexer();
    intake = new Intake();
    pivot = new Pivot();
    swerve = new Swerve(true);

    autoFactory = new AutoFactory(null, null, null, false, swerve);
    autoChooser = new AutoChooser();

    autoChooser.addRoutine("move", () -> move());
    autoChooser.addRoutine("shoot", () -> shoot());
    SmartDashboard.putData("auto choices",autoChooser);

    controller = new XboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.driveFieldRelativeCommand(controller::getLeftY, controller::getLeftX, controller::getRightX));
    
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
            shooter.setVelocityCommand(ShooterConstants.SHOOT_SPEED),
            indexer.setSpeedCommand(-IndexerConstants.SHOOTING_SPEED)));
    shootTrigger.onFalse(
        Commands.parallel(shooter.setVelocityCommand(RPM.of(0)), indexer.setSpeedCommand(0)));
  }

  private AutoRoutine move() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory move = routine.trajectory("move");

    routine.active().onTrue(Commands.sequence(move.resetOdometry(), move.cmd()));

    return routine;
  }

  private AutoRoutine shoot() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    routine.active().onTrue(
      Commands.sequence(
        Commands.parallel(
          shooter.setVelocityCommand(ShooterConstants.SHOOT_SPEED),
          indexer.setSpeedCommand(-IndexerConstants.SHOOTING_SPEED)), Commands.waitSeconds(1),
          shooter.setVelocityCommand(RPM.of(0)),
          indexer.setSpeedCommand(0)));

    return routine;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
