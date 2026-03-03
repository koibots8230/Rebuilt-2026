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
import frc.robot.Constants.PivotConstants;
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

    chooser.addRoutine("P3 Depot", () -> P2_Depot(shooter, indexer, intake, pivot));
    chooser.addRoutine(
        "P3 Depot & Climb", () -> P2_Depot_Climb(shooter, indexer, intake, pivot, climber));
    chooser.addRoutine("P4 Shoot", () -> P3_Shoot(shooter, indexer));
    chooser.addRoutine("P4 Shoot & Climb", () -> P3_Shoot_Climb(shooter, indexer, climber));
    chooser.addRoutine("P5 Shoot", () -> P4_Shoot(shooter, indexer));
    chooser.addRoutine("P5 Shoot & Climb", () -> P4_Shoot_Climb(shooter, indexer, climber));

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

  private AutoRoutine P3_Shoot(Shooter shooter, Indexer indexer) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive = routine.trajectory("P3_Shoot");

    routine.active().onTrue(Commands.sequence(drive.resetOdometry(), drive.cmd()));

    drive.done().onTrue(ShootCommands.autoShoot(shooter, indexer, AutoConstants.SHOOT_TIME_LONG));

    return routine;
  }

  private AutoRoutine P3_Shoot_Climb(Shooter shooter, Indexer indexer, Climber climber) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive1 = routine.trajectory("P3_Shoot");
    AutoTrajectory drive2 = routine.trajectory("P3_Shoot_Climb");

    routine.active().onTrue(Commands.sequence(drive1.resetOdometry(), drive1.cmd()));

    drive1
        .done()
        .onTrue(
            Commands.sequence(
                Commands.parallel(
                    ShootCommands.autoShoot(shooter, indexer, AutoConstants.SHOOT_TIME_LONG),
                    climber.raiseClimbCommand()),
                drive2.cmd()));

    drive2.done().onTrue(climber.lowerClimbCommand());

    return routine;
  }

  private AutoRoutine P4_Shoot(Shooter shooter, Indexer indexer) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive = routine.trajectory("P4_Shoot");

    routine.active().onTrue(Commands.sequence(drive.resetOdometry(), drive.cmd()));

    drive.done().onTrue(ShootCommands.autoShoot(shooter, indexer, AutoConstants.SHOOT_TIME_LONG));

    return routine;
  }

  private AutoRoutine P4_Shoot_Climb(Shooter shooter, Indexer indexer, Climber climber) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive1 = routine.trajectory("P4_Shoot");
    AutoTrajectory drive2 = routine.trajectory("P4_Shoot_Climb");

    routine.active().onTrue(Commands.sequence(drive1.resetOdometry(), drive1.cmd()));

    drive1
        .done()
        .onTrue(
            Commands.sequence(
                Commands.parallel(
                    ShootCommands.autoShoot(shooter, indexer, AutoConstants.SHOOT_TIME_LONG),
                    climber.raiseClimbCommand()),
                drive2.cmd()));

    drive2.done().onTrue(climber.lowerClimbCommand());

    return routine;
  }

  private AutoRoutine P2_Depot(Shooter shooter, Indexer indexer, Intake intake, Pivot pivot) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive1 = routine.trajectory("P2_Shoot");
    AutoTrajectory drive2 = routine.trajectory("P2_Depot");

    routine.active().onTrue(Commands.sequence(drive1.resetOdometry(), drive1.cmd()));

    drive1.done().onTrue(Commands.sequence(
        ShootCommands.autoShoot(shooter, indexer, AutoConstants.SHOOT_TIME_SHORT),
        Commands.parallel(
          IntakeCommands.intakeStart(intake, pivot),
          drive2.cmd()
        )
    ));

    drive2.done().onTrue(Commands.parallel(
      intake.setSpeedCommand(0),
      ShootCommands.autoShoot(shooter, indexer, AutoConstants.SHOOT_TIME_LONG)
    ));

    return routine;
  }

  public AutoRoutine P2_Depot_Climb(
      Shooter shooter, Indexer indexer, Intake intake, Pivot pivot, Climber climber) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");
    AutoTrajectory drive1 = routine.trajectory("P2_Shoot");
    AutoTrajectory drive2 = routine.trajectory("P2_Depot");
    AutoTrajectory drive3 = routine.trajectory("P2_Depot_Climb");

    routine.active().onTrue(Commands.sequence(drive1.resetOdometry(), drive1.cmd()));

    drive1.done().onTrue(Commands.sequence(
        ShootCommands.autoShoot(shooter, indexer, AutoConstants.SHOOT_TIME_SHORT),
        Commands.parallel(
          IntakeCommands.intakeStart(intake, pivot),
          drive2.cmd()
        )
    ));

    drive2.done().onTrue(Commands.sequence(
      Commands.parallel(
        intake.setSpeedCommand(0),
        ShootCommands.autoShoot(shooter, indexer, AutoConstants.SHOOT_TIME_SHORT),
        climber.raiseClimbCommand()
      ),
      Commands.parallel(
        pivot.setPositionCommand(PivotConstants.UP_POSITION),
        drive3.cmd()
      )
    ));

    drive3.done().onTrue(climber.lowerClimbCommand());


    return routine;
  }
}
