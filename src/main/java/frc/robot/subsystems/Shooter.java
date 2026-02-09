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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  @NotLogged private final SparkMax flywheelMotor;
  @NotLogged private final SparkMaxConfig flywheelMotorConfig;
  @NotLogged private SparkClosedLoopController flywheelMotorController;
  @NotLogged private final SparkFlex intakeMotor;
  @NotLogged private final SparkFlexConfig intakeMotorConfig;
  @NotLogged private SparkClosedLoopController intakeMotorController;

  private Voltage flywheelVoltage;
  private AngularVelocity flywheelVelocity;
  private Current flywheelCurrent;
  private AngularVelocity flywheelSetpoint;

  private Voltage intakeVoltage;
  private AngularVelocity intakeVelocity;
  private Current intakeCurrent;
  private AngularVelocity intakeSetpoint;

  public Shooter() {
    flywheelMotor = new SparkMax(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
    flywheelMotorConfig = new SparkMaxConfig();
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

    intakeMotor = new SparkFlex(ShooterConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakeMotorConfig = new SparkFlexConfig();
    intakeMotorConfig.closedLoop.p(ShooterConstants.INTAKE_PID.kp);
    intakeMotorConfig.closedLoop.feedForward.kV(ShooterConstants.INTAKE_FEEDFORWARD.kv);

    intakeMotorConfig.smartCurrentLimit((int) ShooterConstants.INTAKE_CURRENT_LIMIT.in(Amps));
    intakeMotorConfig.inverted(true);
    intakeMotor.configure(
        intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeMotorController = intakeMotor.getClosedLoopController();
    intakeVoltage = Volts.of(0);
    intakeVelocity = RPM.of(0);
    intakeCurrent = Amps.of(0);
    intakeSetpoint = RPM.of(0);
  }

  @Override
  public void periodic() {
    flywheelVoltage = Volts.of(flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage());
    flywheelVelocity = RPM.of(flywheelMotor.getEncoder().getVelocity());
    flywheelCurrent = Amps.of(flywheelMotor.getOutputCurrent());

    intakeVoltage = Volts.of(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    intakeVelocity = RPM.of(intakeMotor.getEncoder().getVelocity());
    intakeCurrent = Amps.of(intakeMotor.getOutputCurrent());
  }

  private void shoot(AngularVelocity intakeVelocity, AngularVelocity flywheelVelocity) {
    flywheelMotorController.setSetpoint(flywheelVelocity.in(RPM), ControlType.kVelocity);
    intakeMotorController.setSetpoint(intakeVelocity.in(RPM), ControlType.kVelocity);
    flywheelSetpoint = flywheelVelocity;
    intakeSetpoint = intakeVelocity;
  }

  public void setupLiveTuning() {
    SmartDashboard.putNumber("Shooter/flywheelkp", ShooterConstants.FLYWHEEL_PID.kp);
    SmartDashboard.putNumber("Shooter/flywheelffkv", ShooterConstants.FLYWHEEL_FEEDFORWARD.kv);
    SmartDashboard.putNumber("Shooter/intakekp", ShooterConstants.INTAKE_PID.kp);
    SmartDashboard.putNumber("Shooter/intakeffkv", ShooterConstants.INTAKE_FEEDFORWARD.kv);
  }

  public void updateLiveTuning() {
    flywheelMotorConfig.closedLoop.p(SmartDashboard.getNumber("Shooter/flywheelkp", ShooterConstants.FLYWHEEL_PID.kp));
    flywheelMotorConfig.closedLoop.feedForward.kV(SmartDashboard.getNumber("Shooter/flywheelffkv", ShooterConstants.FLYWHEEL_FEEDFORWARD.kv));
    intakeMotorConfig.closedLoop.p(SmartDashboard.getNumber("Shooter/intakekp", ShooterConstants.INTAKE_PID.kp));
    intakeMotorConfig.closedLoop.feedForward.kV(SmartDashboard.getNumber("Shooter/intakeffkv", ShooterConstants.INTAKE_FEEDFORWARD.kv));

    flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command setVelocityCommand(
      AngularVelocity intakeVelocity, AngularVelocity flywheelVelocity) {
    return Commands.runOnce(() -> shoot(intakeVelocity, flywheelVelocity), this);
  }
}
