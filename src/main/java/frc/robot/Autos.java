package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Swerve;

public class Autos {

  private AutoFactory factory;
  private AutoChooser chooser;

  Autos(Swerve swerve) {

    factory =
        new AutoFactory(
            swerve::getEstPos, swerve::resetOdometry, swerve::followTrajectory, true, swerve);
    chooser = new AutoChooser();

    chooser.addRoutine("simple sample auto", this::simpleAuto);

    SmartDashboard.putData("hi", chooser);
    RobotModeTriggers.autonomous().whileTrue(chooser.selectedCommandScheduler());
  }

  private AutoRoutine simpleAuto() {
    AutoRoutine routine = factory.newRoutine("taxi");
    AutoTrajectory move = routine.trajectory("simpleAuto");

    routine.active().onTrue(Commands.sequence(move.resetOdometry(), move.cmd()));

    return routine;
  }
}
