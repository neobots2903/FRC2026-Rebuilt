package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class climbIOsparkmax implements climbIO {
  // The climb motor
  private final SparkMax climbMotor;
  private final RelativeEncoder climbEncoder;
  private final SparkClosedLoopController climbPID;
  private Double climbSetpointDegrees = 0.0;

  // Generally sets up the motors
  @SuppressWarnings("removal")
  public climbIOsparkmax() {
    // Initializes the climb motor
    climbMotor = new SparkMax(climbConstants.kClimbMotorId, MotorType.kBrushless);
    // Configures the motor
    climbMotor.configure(
        configureClimbMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climbEncoder = climbMotor.getEncoder();
    climbPID = climbMotor.getClosedLoopController();
  }
  // Configures the climb motor more
  private SparkMaxConfig configureClimbMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(climbConstants.kClimbCurrentLimit);
    config.voltageCompensation(12.0);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // Configures PID
    config.closedLoop.pid(climbConstants.kClimbP, climbConstants.kClimbI, climbConstants.kClimbD);
    // Configures encoder conversion factors, converting from rotations to degrees
    config.encoder.positionConversionFactor(climbConstants.kClimbDegreesPerRotation);
    // Sets soft limits for the climb rotation, stopping it but allowing room for error
    config.softLimit.forwardSoftLimitEnabled(true);
    config.softLimit.reverseSoftLimitEnabled(true);
    config.softLimit.forwardSoftLimit(climbConstants.kOut + 5.0);
    config.softLimit.reverseSoftLimit(climbConstants.kIn - 5.0);
    return config;
  }

  @Override
  // Updates inputs for the climb system
  public void updateInputs(climbIOInputs inputs) {
    // Updates the climb inputs
    inputs.climbRotationDegrees = climbEncoder.getPosition();
    inputs.climbAppliedCurrentAmps = climbMotor.getOutputCurrent();
    inputs.climbAppliedVolts = climbMotor.getAppliedOutput() * climbMotor.getBusVoltage();
    inputs.climbSetRotationDegrees = climbSetpointDegrees;
  }

  @Override
  // Sets the rotation for the climb
  public void setClimbRotationDegrees(double rotationDegrees) {
    double clampedPosition =
        MathUtil.clamp(rotationDegrees, climbConstants.kIn, climbConstants.kOut);
    climbSetpointDegrees = clampedPosition;
    climbPID.setReference(clampedPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  // Stops the system
  public void stop() {
    climbMotor.stopMotor();
  }

  @Override
  // Resets the encoders
  public void resetEncoders() {
    climbEncoder.setPosition(0.0);
  }

  @Override
  // Sets the brake mode
  public void setBrakeMode(boolean enabled) {
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    IdleMode mode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
    climbConfig.idleMode(mode);
    climbMotor.configure(
        climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
