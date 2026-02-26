package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

  @NotLogged private final SparkMax feederMotor;
  @NotLogged private final SparkMaxConfig feederMotorConfig;
  @NotLogged private final SparkClosedLoopController feederMotorController;
  

  private Voltage flywheelVoltage;
  private AngularVelocity flywheelVelocity;
  private Current flywheelCurrent;
  private AngularVelocity flywheelSetpoint;

  private Voltage feederVoltage;
  private AngularVelocity feederVelocity;
  private Current feederCurrent;
  private AngularVelocity feederSetpoint;

  public Shooter() {
    flywheelMotor = new SparkFlex(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
    flywheelMotorConfig = new SparkFlexConfig();
    flywheelMotorConfig.closedLoop.p(ShooterConstants.FLYWHEEL_PID.kp);
    flywheelMotorConfig.closedLoop.feedForward.kV(ShooterConstants.FLYWHEEL_FEEDFORWARD.kv);
    flywheelMotorConfig.idleMode(IdleMode.kCoast);

    flywheelMotorConfig.smartCurrentLimit((int) ShooterConstants.FLYWHEEL_CURRENT_LIMIT.in(Amps));
    flywheelMotorConfig.inverted(true);
    flywheelMotor.configure(
        flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flywheelMotorController = flywheelMotor.getClosedLoopController();

    flywheelVoltage = Volts.of(0);
    flywheelVelocity = RPM.of(0);
    flywheelCurrent = Amps.of(0);
    flywheelSetpoint = RPM.of(0);

    feederMotor = new SparkMax(ShooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    feederMotorConfig = new SparkMaxConfig();
    feederMotorConfig.closedLoop.p(ShooterConstants.FEEDER_PID.kp);
    feederMotorConfig.closedLoop.feedForward.kV(ShooterConstants.FEEDER_FEEDFORWARD.kv);
    feederMotorConfig.smartCurrentLimit((int) ShooterConstants.FEEDER_CURRENT_LIMIT.in(Amps));
    feederMotorConfig.inverted(true);
    feederMotorConfig.idleMode(IdleMode.kBrake);
    feederMotor.configure(
        feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    feederMotorController = feederMotor.getClosedLoopController();

    feederVoltage = Volts.of(0);
    feederVelocity = RPM.of(0);
    feederCurrent = Amps.of(0);
    feederSetpoint = RPM.of(0);
  }

  @Override
  public void periodic() {
    flywheelVoltage = Volts.of(flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage());
    flywheelVelocity = RPM.of(flywheelMotor.getEncoder().getVelocity());
    flywheelCurrent = Amps.of(flywheelMotor.getOutputCurrent());

    feederVoltage = Volts.of(feederMotor.getAppliedOutput() * feederMotor.getBusVoltage());
    feederVelocity = RPM.of(feederMotor.getEncoder().getVelocity());
    feederCurrent = Amps.of(feederMotor.getOutputCurrent());

  }

  @Override
  public void simulationPeriodic() {
    flywheelVelocity = flywheelSetpoint;
  }

  private void shoot(AngularVelocity flywheelVelocity, AngularVelocity feederVelocity) {
    flywheelMotorController.setSetpoint(flywheelVelocity.in(RPM), ControlType.kVelocity);
    flywheelSetpoint = flywheelVelocity;
    feederMotorController.setSetpoint(feederVelocity.in(RPM), ControlType.kVelocity);
    feederSetpoint = feederVelocity;
  }

  public Command setVelocityCommand(AngularVelocity flywheelVelocity, AngularVelocity feederVelocity) {
    return Commands.runOnce(() -> shoot(flywheelVelocity, feederVelocity), this);
  }
}
