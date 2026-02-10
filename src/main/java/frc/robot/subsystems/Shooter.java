package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {

  @NotLogged private final SparkFlex flywheelMotor;
  @NotLogged private final SparkFlexConfig flywheelMotorConfig;
  @NotLogged private final SparkClosedLoopController flywheelMotorController;

  private Voltage flywheelVoltage;
  private AngularVelocity flywheelVelocity;
  private Current flywheelCurrent;
  private AngularVelocity flywheelSetpoint;


  //Reason I deleted all of the intake stuff is because the shooter was reconfigured to not use a seperate motor for the intake.
  //Also, I know Jake hates comments.

  public Shooter() {
    flywheelMotor = new SparkFlex(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
    flywheelMotorConfig = new SparkFlexConfig();
    flywheelMotorConfig.closedLoop.p(ShooterConstants.FLYWHEEL_PID.kp);
    flywheelMotorConfig.closedLoop.feedForward.kV(ShooterConstants.FLYWHEEL_FEEDFORWARD.kv);

    flywheelMotorConfig.smartCurrentLimit((int) ShooterConstants.FLYWHEEL_CURRENT_LIMIT.in(Amps));
    flywheelMotorConfig.inverted(true);
    flywheelMotor.configure(
        flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flywheelMotorController = flywheelMotor.getClosedLoopController();

    flywheelVoltage = Volts.of(0);
    flywheelVelocity = RPM.of(0);
    flywheelCurrent = Amps.of(0);
    flywheelSetpoint = RPM.of(0);
  }

  @Override
  public void periodic() {
    flywheelVoltage = Volts.of(flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage());
    flywheelVelocity = RPM.of(flywheelMotor.getEncoder().getVelocity());
    flywheelCurrent = Amps.of(flywheelMotor.getOutputCurrent());
  }

  @Override
  public void simulationPeriodic() {
    flywheelVelocity = flywheelSetpoint;
  }

  private void shoot(AngularVelocity flywheelVelocity) {
    flywheelMotorController.setSetpoint(flywheelVelocity.in(RPM), ControlType.kVelocity);
    flywheelSetpoint = flywheelVelocity;
  }

  public Command setVelocityCommand(AngularVelocity flywheelVelocity) {
    return Commands.runOnce(() -> shoot(flywheelVelocity), this);
  }
}
