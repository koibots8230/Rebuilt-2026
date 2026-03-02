package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;

public class ShootCommands {
  public static Command shoot(Shooter shooter, Indexer indexer, Pivot pivot) {
    return Commands.parallel(
        shooter.setVelocityCommand(ShooterConstants.FLYWHEEL_SPEED, ShooterConstants.FEEDER_SPEED),
        indexer.setSpeedCommand(IndexerConstants.SHOOTING_SPEED),
        Commands.sequence(
          pivot.setPositionCommand(PivotConstants.MID_POSITION),
          Commands.waitUntil(pivot::atPosition),
          pivot.setPositionCommand(PivotConstants.DOWN_POSITION),
          Commands.waitUntil(pivot::atPosition)
        ).repeatedly()
    );
  }

  public static Command stop(Shooter shooter, Indexer indexer, Pivot pivot) {
    return Commands.parallel(
        shooter.setVelocityCommand(Units.RPM.of(0), 
        Units.RPM.of(0)), indexer.setSpeedCommand(0),
        pivot.setPositionCommand(PivotConstants.DOWN_POSITION)
    );
  }



  public static Command autoShoot(Shooter shooter, Indexer indexer, Pivot pivot, Time time) {
    return Commands.sequence(
        shoot(shooter, indexer, pivot), Commands.waitTime(time), stop(shooter, indexer, pivot));
  }
}
