package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;

public class ShootCommands {
  public static Command shoot(Shooter shooter, Indexer indexer) {
    return Commands.parallel(
        shooter.setVelocityCommand(ShooterConstants.FEEDER_SPEED, ShooterConstants.FLYWHEEL_SPEED),
        indexer.setSpeedCommand(IndexerConstants.SHOOTING_SPEED));
  }

  public static Command stop(Shooter shooter, Indexer indexer) {
    return Commands.parallel(
        shooter.setVelocityCommand(Units.RPM.of(0), Units.RPM.of(0)), indexer.setSpeedCommand(0));
  }

  public static Command autoShoot(Shooter shooter, Indexer indexer, Time time) {
    return Commands.sequence(
        shoot(shooter, indexer), Commands.waitTime(time), stop(shooter, indexer));
  }
}
