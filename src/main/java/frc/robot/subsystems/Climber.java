
@Logged
public class Climber extends SubsystemBase {
    @NotLogged private final SparkMax motor;
    @NotLogged private final SparkMaxConfig config;
    @NotLogged private final RelativeEncoder encoder;

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

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ClimberConstants.CURRENT_LIMIT.in(Amps));
        config.inverted(false);

        encoder = motor.getRelativeEncoder();

        current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
        voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
        setpoint = ClimberConstants.DOWN_POSITION;

        position = encoder.getPosition();
        velocity = encoder.getVelocity();

    }

}