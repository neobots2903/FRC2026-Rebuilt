package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class intakeIOsparkmax implements intakeIO {

  // The pivot and intake motors
  private final SparkMax pivotMotor;
  private final SparkMax intakeMotor;
  private final RelativeEncoder pivotEncoder;
  private final SparkClosedLoopController pivotPID;
  private Double pivotSetpointDegrees = 0.0;

  // Generally sets up the motors
  @SuppressWarnings("removal")
  public intakeIOsparkmax() {
    // intakeConstants does not exist yet
    // Initializes the pivot motor
    pivotMotor = new SparkMax(intakeConstants.kPivotMotorId, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getEncoder();
    pivotPID = pivotMotor.getClosedLoopController();
    // Initializes the intake motor
    intakeMotor = new SparkMax(intakeConstants.kIntakeMotorId, MotorType.kBrushless);
    // Configures both motors
    pivotMotor.configure(
        configurePivotMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(
        configureIntakeMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  // Configures the pivot motor
  private SparkMaxConfig configurePivotMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(intakeConstants.kPivotCurrentLimit);
    config.voltageCompensation(12.0);
    // Configures PID
    config.closedLoop.pid(
        intakeConstants.kPivotP, intakeConstants.kPivotI, intakeConstants.kPivotD);
    // Configures encoder conversion factors, converting from rotations to degrees
    config.encoder.positionConversionFactor(intakeConstants.kPivotDegreesPerRotation);
    // Sets soft limits for the pivot position, stopping it but allowing room for error
    config.softLimit.forwardSoftLimitEnabled(true);
    config.softLimit.reverseSoftLimitEnabled(true);
    config.softLimit.forwardSoftLimit(intakeConstants.kOutPosition + 5.0);
    config.softLimit.reverseSoftLimit(intakeConstants.kInPosition - 5.0);
    return config;
  }
  // Configures the intake motor
  private SparkMaxConfig configureIntakeMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(intakeConstants.kIntakeCurrentLimit);
    config.voltageCompensation(12.0);
    return config;
  }

  @Override
  // Updates inputs for both systems
  public void updateInputs(intakeIOInputs inputs) {
    // Updates pivot inputs
    inputs.pivotPositionDegrees = pivotEncoder.getPosition();
    inputs.pivotAppliedCurrentAmps = pivotMotor.getOutputCurrent();
    inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.pivotSetPointDegrees = pivotSetpointDegrees;
    // Updates intake inputs
    inputs.intakeAppliedCurrentAmps = intakeMotor.getOutputCurrent();
    inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
  }

  @Override
  // Sets the position for the pivot
  public void setPivotPosition(double positionDegrees) {
    double clampedPosition =
        MathUtil.clamp(positionDegrees, intakeConstants.kInPosition, intakeConstants.kOutPosition);
    pivotSetpointDegrees = clampedPosition;
    pivotPID.setReference(clampedPosition, ControlType.kPosition);
  }

  @Override
  // Sets the voltage for the intake
  public void setIntakeVoltage(double voltage) {
    double clampedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    intakeMotor.setVoltage(clampedVolts);
  }

  @Override
  // Stops the systems
  public void stop() {
    pivotMotor.stopMotor();
    intakeMotor.stopMotor();
  }

  @Override
  // Resets the encoders
  public void resetEncoders() {
    pivotEncoder.setPosition(0.0);
  }

  @Override
  // Sets the brake mode
  public void setBrakeMode(boolean enabled) {
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    IdleMode mode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
    pivotConfig.idleMode(mode);
    intakeConfig.idleMode(mode);
    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
