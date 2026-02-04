package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {

  @NotLogged private final CommandXboxController controller;
  private final Swerve swerve;

  public RobotContainer() {
    controller = new CommandXboxController(0);
    swerve = new Swerve();

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.driveCommand(controller::getLeftY, controller::getLeftX, controller::getRightX));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
