package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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

public class shooterIOreal implements shooterIO {

  // The flywheel and hood motors
  private final TalonFX flywheelMotor;
  private final TalonFX flywheelMotor2;
  private final SparkMax hoodMotor;

  private final RelativeEncoder hoodEncoder;
  private final SparkClosedLoopController hoodPID;

  // Generally sets up the motors
  @SuppressWarnings("removal")
  public shooterIOreal() {
    flywheelMotor = new TalonFX(shooterConstants.kFlywheelMotorID);
    flywheelMotor.getConfigurator().apply(configureFlywheelMotor());
    flywheelMotor2 = new TalonFX(shooterConstants.kFlywheelMotor2ID);
    flywheelMotor2.getConfigurator().apply(configureFlywheelMotor());
    flywheelMotor2.setControl(
        new Follower(flywheelMotor2.getDeviceID(), MotorAlignmentValue.Opposed));
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
    config.Slot0.kP = shooterConstants.kFlywheel2P;
    config.Slot0.kI = shooterConstants.kFlywheel2I;
    config.Slot0.kD = shooterConstants.kFlywheel2D;
    config.CurrentLimits.SupplyCurrentLimit = shooterConstants.kCurrentLimit;
    config.Feedback.SensorToMechanismRatio = shooterConstants.kFlywheelGearRatio;
    return config;
  }

  // Configures the hood motor
  private SparkMaxConfig configureHoodMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(shooterConstants.kCurrentLimit);
    config.closedLoop.pid(
        shooterConstants.kHoodP, shooterConstants.kHoodI, shooterConstants.kHoodD); // Placeholder
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
    flywheelMotor.getVelocity();
    inputs.flywheelVelocity =
        flywheelMotor.getVelocity().getValueAsDouble()
            * 60.0; // Might need to multiply by gear ratio
    flywheelMotor.getStatorCurrent();
    inputs.flywheelCurrent = flywheelMotor.getStatorCurrent().getValueAsDouble();
    flywheelMotor2.getVelocity();
    inputs.flywheel2Velocity =
        flywheelMotor2.getVelocity().getValueAsDouble()
            * 60.0; // Might need to multiply by gear ratio
    flywheelMotor2.getStatorCurrent();
    inputs.flywheel2Current = flywheelMotor2.getStatorCurrent().getValueAsDouble();
    // Updates the hood inputs
    inputs.hoodPositionDegrees = hoodEncoder.getPosition();
    inputs.hoodAppliedCurrentAmps = hoodMotor.getOutputCurrent();
    inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
  }

  @Override
  // Sets the angle for the hood
  public void setHoodPosition(double positionDegrees) {
    double clampedPosition =
        MathUtil.clamp(
            positionDegrees, shooterConstants.kMinHoodAngle, shooterConstants.kMaxHoodAngle);
    hoodPID.setReference(clampedPosition, ControlType.kPosition);
  }

  @Override
  // Sets the velocity for the flywheels
  public void setFlywheelVelocity(double velocityRPM) {
    final VelocityVoltage flywheelMotor_request = new VelocityVoltage(0).withSlot(0);
    flywheelMotor.setControl(
        flywheelMotor_request
            .withVelocity(velocityRPM * 60)
            .withFeedForward(shooterConstants.kFeedForward * Math.signum(velocityRPM)));
    final VelocityVoltage flywheelMotor2_request = new VelocityVoltage(0).withSlot(0);
    flywheelMotor2.setControl(
        flywheelMotor2_request
            .withVelocity(velocityRPM * 60)
            .withFeedForward(shooterConstants.kFeedForward * Math.signum(velocityRPM)));
  }

  @Override
  // Stops the systems
  public void stop() {
    flywheelMotor.stopMotor();
    flywheelMotor2.stopMotor();
    hoodMotor.stopMotor();
  }

  @Override
  // Resets the encoders
  public void resetEncoders() {
    flywheelMotor.setPosition(0.0);
    hoodEncoder.setPosition(0.0);
  }
}
