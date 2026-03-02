package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Autos {

  private final AutoFactory autoFactory;
  private AutoChooser chooser;

  Autos(
      Swerve swerve,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Pivot pivot,
      Climber climber) {
    autoFactory =
        new AutoFactory(
            swerve::getEstimatedPosition,
            swerve::setOdometry,
            swerve::followTrajectory,
            true,
            swerve);
    chooser = new AutoChooser();

    chooser.addRoutine("sample auto", () -> sampleAuto(shooter, indexer));

    chooser.addRoutine("P3 Depot", () -> P3_Depot(shooter, indexer, intake, pivot));
    chooser.addRoutine(
        "P3 Depot & Climb", () -> P3_Depot_Climb(shooter, indexer, intake, pivot, climber));
    chooser.addRoutine("P4 Shoot", () -> P4_Shoot(shooter, indexer, pivot));
    chooser.addRoutine("P4 Shoot & Climb", () -> P4_Shoot_Climb(shooter, indexer, pivot, climber));
    chooser.addRoutine("P5 Shoot", () -> P5_Shoot(shooter, indexer, pivot));
    chooser.addRoutine("P5 Shoot & Climb", () -> P5_Shoot_Climb(shooter, indexer, pivot, climber));

    SmartDashboard.putData("autos", chooser);
    RobotModeTriggers.autonomous().whileTrue(chooser.selectedCommandScheduler());
  }

  public Command getCommandScheduler() {
    return chooser.selectedCommandScheduler();
  }

  private AutoRoutine sampleAuto(Shooter shooter, Indexer indexer) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory move = routine.trajectory("sampleAuto");

    routine.active().onTrue(Commands.sequence(move.resetOdometry(), move.cmd()));

    return routine;
  }

  private AutoRoutine P4_Shoot(Shooter shooter, Indexer indexer, Pivot pivot) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive = routine.trajectory("P4_Shoot");

    routine.active().onTrue(Commands.sequence(drive.resetOdometry(), drive.cmd()));

    drive.done().onTrue(ShootCommands.autoShoot(shooter, indexer, pivot, AutoConstants.SHOOT_TIME_LONG));

    return routine;
  }

  private AutoRoutine P4_Shoot_Climb(Shooter shooter, Indexer indexer, Pivot pivot, Climber climber) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive1 = routine.trajectory("P4_Shoot");
    AutoTrajectory drive2 = routine.trajectory("P4_Shoot_Climb");

    routine.active().onTrue(Commands.sequence(drive1.resetOdometry(), drive1.cmd()));

    drive1
        .done()
        .onTrue(
            Commands.sequence(
                Commands.parallel(
                    ShootCommands.autoShoot(shooter, indexer, pivot, AutoConstants.SHOOT_TIME_LONG),
                    climber.raiseClimbCommand()),
                drive2.cmd()));

    drive2.done().onTrue(climber.lowerClimbCommand());

    return routine;
  }

  private AutoRoutine P5_Shoot(Shooter shooter, Indexer indexer, Pivot pivot) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive = routine.trajectory("P5_Shoot");

    routine.active().onTrue(Commands.sequence(drive.resetOdometry(), drive.cmd()));

    drive.done().onTrue(ShootCommands.autoShoot(shooter, indexer, pivot, AutoConstants.SHOOT_TIME_LONG));

    return routine;
  }

  private AutoRoutine P5_Shoot_Climb(Shooter shooter, Indexer indexer, Pivot pivot, Climber climber) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive1 = routine.trajectory("P5_Shoot");
    AutoTrajectory drive2 = routine.trajectory("P5_Shoot_Climb");

    routine.active().onTrue(Commands.sequence(drive1.resetOdometry(), drive1.cmd()));

    drive1
        .done()
        .onTrue(
            Commands.sequence(
                Commands.parallel(
                    ShootCommands.autoShoot(shooter, indexer, pivot, AutoConstants.SHOOT_TIME_LONG),
                    climber.raiseClimbCommand()),
                drive2.cmd()));

    drive2.done().onTrue(climber.lowerClimbCommand());

    return routine;
  }

  private AutoRoutine P3_Depot(Shooter shooter, Indexer indexer, Intake intake, Pivot pivot) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive1 = routine.trajectory("P3_Depot1");
    AutoTrajectory drive2 = routine.trajectory("P3_Depot2");
    AutoTrajectory drive3 = routine.trajectory("P3_Depot3");

    routine.active().onTrue(Commands.sequence(drive1.resetOdometry(), drive1.cmd()));

    drive1
        .done()
        .onTrue(
            Commands.sequence(
                ShootCommands.autoShoot(shooter, indexer, pivot, AutoConstants.SHOOT_TIME_SHORT),
                drive2.cmd()));

    drive2
        .done()
        .onTrue(
            Commands.sequence(
                IntakeCommands.autoIntake(intake, pivot, AutoConstants.DEPOT_INTAKE_TIME),
                drive3.cmd()));

    drive3.done().onTrue(ShootCommands.autoShoot(shooter, indexer, pivot, AutoConstants.SHOOT_TIME_LONG));

    return routine;
  }

  public AutoRoutine P3_Depot_Climb(
      Shooter shooter, Indexer indexer, Intake intake, Pivot pivot, Climber climber) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive1 = routine.trajectory("P3_Depot1");
    AutoTrajectory drive2 = routine.trajectory("P3_Depot2");
    AutoTrajectory drive3 = routine.trajectory("P3_Depot3");
    AutoTrajectory drive4 = routine.trajectory("P3_Depot_Climb");

    routine.active().onTrue(Commands.sequence(drive1.resetOdometry(), drive1.cmd()));

    drive1
        .done()
        .onTrue(
            Commands.sequence(
                ShootCommands.autoShoot(shooter, indexer, pivot, AutoConstants.SHOOT_TIME_SHORT),
                drive2.cmd()));

    drive2
        .done()
        .onTrue(
            Commands.sequence(
                IntakeCommands.autoIntake(intake, pivot, AutoConstants.DEPOT_INTAKE_TIME),
                drive3.cmd()));

    drive3
        .done()
        .onTrue(
            Commands.sequence(
                Commands.parallel(
                    ShootCommands.autoShoot(shooter, indexer, pivot, AutoConstants.SHOOT_TIME_LONG),
                    climber.raiseClimbCommand()),
                drive4.cmd()));

    drive4.done().onTrue(climber.lowerClimbCommand());

    return routine;
  }
}
