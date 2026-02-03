package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Shooter;

@Logged
@Logged
public class RobotContainer {

  @NotLogged private final CommandXboxController controller;
  private final Shooter shooter;

  public RobotContainer() {
    shooter = new Shooter();
    controller = new CommandXboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    Trigger shootTrigger = new Trigger(() -> controller.getRightTriggerAxis() > 0.15);
    shootTrigger.whileTrue(shooter.setVelocityCommand(Constants.ShooterConstants.SHOOT_SPEED));
    shootTrigger.onFalse(shooter.setVelocityCommand(edu.wpi.first.units.Units.RPM.of(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
