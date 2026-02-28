package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Autos {

  private AutoFactory factory;
  private AutoChooser chooser;

  Autos(Swerve swerve, Shooter shooter, Indexer indexer) {
    factory =
        new AutoFactory(
            swerve::getEstPos, swerve::resetOdometry, swerve::followTrajectory, true, swerve);
    chooser = new AutoChooser();

    chooser.addRoutine("sample auto", () -> sampleAuto(shooter, indexer));

    SmartDashboard.putData("hi", chooser);
    RobotModeTriggers.autonomous().whileTrue(chooser.selectedCommandScheduler());
  }

  private AutoRoutine sampleAuto(Shooter shooter, Indexer indexer) {
    AutoRoutine routine = factory.newRoutine("taxi");
    AutoTrajectory move = routine.trajectory("simpleAuto");

    routine.active().onTrue(Commands.sequence(
      move.resetOdometry(), 
      move.cmd()
    ));

    return routine;
  }

  private AutoRoutine P4_Shoot(Shooter shooter, Indexer indexer) {
    AutoRoutine routine = factory.newRoutine("taxi");
    AutoTrajectory drive = routine.trajectory("P4_Shoot");

    routine.active().onTrue(Commands.sequence(
      drive.resetOdometry(), 
      drive.cmd()
    ));

    drive.done().onTrue(ShootCommands.autoShoot(shooter, indexer));

    return routine;
  }

  private AutoRoutine P4_Shoot_Climb(Shooter shooter, Indexer indexer, Climber climber) {
    AutoRoutine routine = factory.newRoutine("taxi");
    AutoTrajectory drive1 = routine.trajectory("P4_Shoot");
    AutoTrajectory drive2 = routine.trajectory("P4_Shoot_Climb");

    routine.active().onTrue(Commands.sequence(
      drive1.resetOdometry(), 
      drive1.cmd()
    ));

    drive1.done().onTrue(Commands.sequence(
      Commands.parallel(
        ShootCommands.autoShoot(shooter, indexer),
        climber.raiseClimbCommand()
      ),
      drive2.cmd()
    ));

    drive2.done().onTrue(climber.lowerClimbCommand());

    return routine;
  }

  private AutoRoutine P5_Shoot(Shooter shooter, Indexer indexer) {
    AutoRoutine routine = factory.newRoutine("taxi");
    AutoTrajectory drive = routine.trajectory("P5_Shoot");

    routine.active().onTrue(Commands.sequence(
      drive.resetOdometry(), 
      drive.cmd()
    ));

    drive.done().onTrue(ShootCommands.autoShoot(shooter, indexer));

    return routine;
  }

  private AutoRoutine P5_Shoot_Climb(Shooter shooter, Indexer indexer, Climber climber) {
    AutoRoutine routine = factory.newRoutine("taxi");
    AutoTrajectory drive1 = routine.trajectory("P5_Shoot");
    AutoTrajectory drive2 = routine.trajectory("P5_Shoot_Climb");

    routine.active().onTrue(Commands.sequence(
      drive1.resetOdometry(), 
      drive1.cmd()
    ));

    drive1.done().onTrue(Commands.sequence(
      Commands.parallel(
        ShootCommands.autoShoot(shooter, indexer),
        climber.raiseClimbCommand()
      ),
      drive2.cmd()
    ));

    drive2.done().onTrue(climber.lowerClimbCommand());

    return routine;
  }

}
