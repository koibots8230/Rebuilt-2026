package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class ShooterHood extends SubsystemBase {
  @NotLogged private final SparkMax pivotMotor;
  @NotLogged private final SparkMaxConfig pivotMotorConfig;
  @NotLogged private final AbsoluteEncoder pivotEncoder;
  @NotLogged private final SparkClosedLoopController pivotController;

  @NotLogged private final TrapezoidProfile pivotTProfile;
  @NotLogged private TrapezoidProfile.State pivotGoal;
  @NotLogged private TrapezoidProfile.State pivotSetpoint;
  @NotLogged private final ArmFeedforward feedforward;

  private double voltage;
  private double current;
  private double position;
  private double setpoint;

  private double calculatedHoodAngle;
  private boolean isBlue;
  private boolean withinHubBounds;

  @SuppressWarnings("removal")
  public ShooterHood() {
    pivotMotor = new SparkMax(HoodConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
    pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig.smartCurrentLimit((int) HoodConstants.CURRENT_LIMIT.in(Amps));
    pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pivotMotorConfig.absoluteEncoder.positionConversionFactor(HoodConstants.CONVERSION_FACTOR);
    pivotMotorConfig.closedLoop.p(HoodConstants.PID.kp);
    pivotMotor.configure(
        pivotMotorConfig,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);
    pivotGoal = new TrapezoidProfile.State(0, 0);
    pivotTProfile =
        new TrapezoidProfile(
            new Constraints(
                HoodConstants.MAX_VELOCITY.in(RadiansPerSecond),
                HoodConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    feedforward =
        new ArmFeedforward(
            HoodConstants.FEEDFORWARD.ks,
            HoodConstants.FEEDFORWARD.kg,
            HoodConstants.FEEDFORWARD.kv);
    pivotController = pivotMotor.getClosedLoopController();
    pivotSetpoint = new TrapezoidProfile.State(0, 0);

    calculatedHoodAngle = 0;
  }

  @Override
  public void periodic() {
    pivotSetpoint =
        pivotTProfile.calculate(RobotConstants.CLOCK_SPEED.in(Seconds), pivotSetpoint, pivotGoal);
    pivotController.setSetpoint(
        pivotSetpoint.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(pivotSetpoint.position, pivotSetpoint.velocity));
    position = pivotEncoder.getPosition();
    current = pivotMotor.getOutputCurrent();
    voltage = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    
  }

  @Override
  public void simulationPeriodic() {
    position = pivotSetpoint.position;
  }

  private void setPosition(double target) {
    pivotGoal = new TrapezoidProfile.State(target, 0);
    setpoint = target;
  }

  public void setIsBlue(boolean color){
    isBlue = color;
  }

  private double calculateHoodAngle(Pose2d estimatedPose) {
    Double FieldXPosition = (estimatedPose.getMeasureX().times((isBlue ? -1 : 1))).in(Inches);
    if (FieldXPosition <= (158.6 - 13.75)){
      withinHubBounds = true;
    } else {
      withinHubBounds = false;
    }
    
    if (withinHubBounds){
      //Run calculations for hood angle and return the result
    } else {
      return 0;
    }
  }

  public Command setPositionCommand(double target) {
    return Commands.runOnce(() -> setPosition(target), this);
  }

  public Command autoSetPositionCommand(Pose2d estimatedPose) {
    return Commands.runOnce(() -> {
      estimatedAngle = calculateHoodAngle(estimatedPose);
      setPosition(estimatedAngle);
    }, this);
  }

  public void setupLiveTuning() {
    SmartDashboard.putNumber()
  }
}
