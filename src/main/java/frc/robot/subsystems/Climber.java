package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class Climber extends SubsystemBase {
    @NotLogged private final SparkMax motor;
    @NotLogged private final SparkClosedLoopController controller;
    @NotLogged private final SparkMaxConfig config;
    @NotLogged private final RelativeEncoder encoder;
    @NotLogged private final TrapezoidProfile profile;

    private Current current;
    private Voltage voltage;
    private double velocity;
    private double setpoint;
    private double position;

    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State motorSetpoint;
    
    public Climber() {
        motor = new SparkMax(ClimberConstants.MOTOR_ID, MotorType.kBrushless);
        config = new SparkMaxConfig();

        config.closedLoop.p(ClimberConstants.CLIMBER_PID.kp);
        config.closedLoop.feedForward.kV(ClimberConstants.CLIMBER_FF.kv);

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit((int) ClimberConstants.CURRENT_LIMIT.in(Amps));
        config.inverted(false);

        encoder = motor.getEncoder();

        current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
        voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
        setpoint = ClimberConstants.DOWN_POSITION;

        position = encoder.getPosition();
        velocity = encoder.getVelocity();

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ClimberConstants.VELOCITY_CONSTRAINT, ClimberConstants.ACCELERATION_CONSTRAINT));
        goal = new TrapezoidProfile.State(0,0);
        motorSetpoint = new TrapezoidProfile.State(0,0);

        controller = motor.getClosedLoopController();


    
    }

    public void periodic() {
        motorSetpoint = profile.calculate(RobotConstants.CLOCK, motorSetpoint, goal);

        controller.setSetpoint(
            motorSetpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0);

    }

}

    /* todo
     * buttons
     * change goal state
     */