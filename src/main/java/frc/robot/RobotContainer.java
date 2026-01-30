package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.PS5Controller;


@Logged
public class RobotContainer {

  private final PS5Controller controller;
  
  public RobotContainer() {
    controller = new PS5Controller(0);

    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

  /* todo make cases (commands)
   * Up D-Pad raise.
   * Down D-Pad lower.
   */
