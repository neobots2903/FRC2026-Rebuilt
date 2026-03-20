package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class shooterIOreal implements shooterIO {

  // The flywheel and hood motors
  private final TalonFX flywheelMotor;
  private final TalonFX flywheelMotor2;
  private final SparkMax hoodMotor;

  private final RelativeEncoder hoodEncoder;
  private final SparkClosedLoopController hoodPID;

  private double hoodSetPoint = 0.0;
  private double flywheelSetPointRPM = 0.0;

  // Generally sets up the motors
  @SuppressWarnings("removal")
  public shooterIOreal() {
    flywheelMotor = new TalonFX(shooterConstants.kFlywheelMotorID);
    flywheelMotor.getConfigurator().apply(configureFlywheelMotor());
    flywheelMotor2 = new TalonFX(shooterConstants.kFlywheelMotor2ID);
    flywheelMotor2.getConfigurator().apply(configureFlywheelMotor2());
    flywheelMotor2.setControl(
        new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    hoodMotor = new SparkMax(shooterConstants.kHoodMotorID, SparkLowLevel.MotorType.kBrushless);
    hoodMotor.configure(
        configureHoodMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodEncoder = hoodMotor.getEncoder();
    hoodPID = hoodMotor.getClosedLoopController();
  }

  // Configures the flywheel motor
  private TalonFXConfiguration configureFlywheelMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = shooterConstants.kFlywheelP;
    config.Slot0.kI = shooterConstants.kFlywheelI;
    config.Slot0.kD = shooterConstants.kFlywheelD;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = shooterConstants.kFlywheelCurrentLimit;
    config.Feedback.SensorToMechanismRatio = shooterConstants.kFlywheelGearRatio;
    return config;
  }

  // Configures the flywheel 2 motor
  private TalonFXConfiguration configureFlywheelMotor2() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = shooterConstants.kFlywheelCurrentLimit;
    config.Feedback.SensorToMechanismRatio = shooterConstants.kFlywheelGearRatio;
    return config;
  }

  // Configures the hood motor
  private SparkMaxConfig configureHoodMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(shooterConstants.kHoodCurrentLimit);
    config.voltageCompensation(12.0); // Placeholder
    config.encoder.positionConversionFactor(shooterConstants.kHoodDegreesPerRotation);
    config.closedLoop.p(shooterConstants.kHoodP);
    config.closedLoop.i(shooterConstants.kHoodI);
    config.closedLoop.d(shooterConstants.kHoodD);
    return config;
  }

  @Override
  // Updates inputs
  public void updateInputs(shooterIOInputs inputs) {
    // Updates the flywheel inputs
    inputs.flywheelVelocity = flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.flywheelCurrent = flywheelMotor.getTorqueCurrent().getValueAsDouble();
    inputs.flywheel2Velocity = flywheelMotor2.getVelocity().getValueAsDouble() * 60.0;
    inputs.flywheel2Current = flywheelMotor2.getTorqueCurrent().getValueAsDouble();
    inputs.flywheelSetpointRPM = flywheelSetPointRPM;
    // Updates the hood inputs — convert encoder degrees to physical launch angle
    inputs.hoodPositionDegrees = hoodEncoder.getPosition() + shooterConstants.kHoodAngleOffset;
    inputs.hoodAppliedCurrentAmps = hoodMotor.getOutputCurrent();
    inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
    inputs.hoodSetPointDegrees = hoodSetPoint;
  }

  @Override
  // Sets the angle for the hood (positionDegrees is physical launch angle, e.g. 20–35)
  public void setHoodPosition(double positionDegrees) {
    this.hoodSetPoint = positionDegrees;
    double clampedPosition =
        MathUtil.clamp(
            positionDegrees, shooterConstants.kMinHoodAngle, shooterConstants.kMaxHoodAngle);
    Logger.recordOutput("Shooter/Hood/SetPoint", clampedPosition);

    // Convert physical angle to encoder space for the onboard PID
    double encoderTarget = clampedPosition - shooterConstants.kHoodAngleOffset;
    hoodPID.setReference(encoderTarget, ControlType.kPosition);
  }

  @Override
  // Sets the velocity for the flywheels
  public void setFlywheelVelocity(double velocityRPM) {
    this.flywheelSetPointRPM = velocityRPM;
    final VelocityVoltage flywheelMotor_request = new VelocityVoltage(0).withSlot(0);
    flywheelMotor.setControl(
        flywheelMotor_request
            .withVelocity(velocityRPM / 60)
            .withFeedForward(shooterConstants.kFeedForward * Math.signum(velocityRPM)));
  }

  @Override
  // Stops the systems
  public void stop() {
    flywheelMotor.stopMotor();
    hoodMotor.stopMotor();
    flywheelSetPointRPM = 0.0;
    hoodSetPoint = shooterConstants.kMinHoodAngle; // Park at physical minimum, not 0
  }

  @Override
  // Resets the encoders
  public void resetEncoders() {
    flywheelMotor.setPosition(0.0);
    hoodEncoder.setPosition(0.0);
  }
}
