package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

public class shooterIOreal implements shooterIO {
   
    // The flywheel, hood, and rotation motors
    private final TalonFX flywheelMotor; // TODO: configure
    private final SparkMax hoodMotor;
    private final SparkMax rotationMotor; // TODO: Remove

    // TODO: Missing encoders and PID controllers.

    // Generally sets up the motors
    @SuppressWarnings("removal")
    public shooterIOreal() {
        // !Initialize flywheelMotor
        hoodMotor = new SparkMax(shooterConstants.kHoodMotorID, MotorType.kBrushless);
        rotationMotor = new SparkMax(shooterConstants.kRotationMotorID, MotorType.kBrushless);
        // !Configure flywheelMotor
        hoodMotor.configure(configureHoodMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotationMotor.configure(configureRotationMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    // !Configure flywheelMotor
    
    // TODO: These two methods are indentical. Refactor to remove redundancy.
    // Configures the hood motor
    private SparkMaxConfig configureHoodMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(shooterConstants.kCurrentLimit);
        config.voltageCompensation(12.0); // Placeholder
        return config;
    }

    // Configures the rotation motor
    private SparkMaxConfig configureRotationMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(shooterConstants.kCurrentLimit);
        config.voltageCompensation(12.0); // Placeholder
        return config;
    }

    @Override
    // Updates inputs
    public void updateInputs(shooterIOInputs inputs) {
        // !Update flywheelMotor inputs

        // Updates the hood inputs
        inputs.hoodPositionDegrees = hoodEncoder.getPosition();
        inputs.hoodAppliedCurrentAmps = hoodMotor.getOutputCurrent();
        inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
        // Updates the rotation inputs
        inputs.rotationPositionDegrees = rotationEncoder.getPosition();
        inputs.rotationAppliedCurrentAmps = rotationMotor.getOutputCurrent();
        inputs.rotationAppliedVolts = rotationMotor.getAppliedOutput() * rotationMotor.getBusVoltage();
    }

    @Override
    // Sets the angle for the hood
    public void setHoodPosition(double positionDegrees) {
        double clampedPosition = 
            MathUtil.clamp(positionDegrees, shooterConstants.kMinHoodAngle, shooterConstants.kMaxHoodAngle);
        hoodSetpointDegrees = clampedPosition;
        hoodPID.setReference(clampedPosition, ControlType.kPosition);
    }

    @Override // TODO: Remove
    // Sets the angle for the rotation
    public void setRotationPosition (double positionDegrees) {
        double clampedPosition = 
            MathUtil.clamp(positionDegrees, shooterConstants.kMinRotationAngle, shooterConstants.kMaxRotationAngle);
        rotationSetpointDegrees = clampedPosition;
        rotationPID.setReference(clampedPosition, ControlType.kPosition);
    }

    @Override
    // Stops the systems
    public void stop() {
        // !Stop flywheelMotor
        hoodMotor.stopMotor();
        rotationMotor.stopMotor();
    }

    @Override
    // Resets the encoders
    public void resetEncoders() {
        // !Reset flywheelMotor encoders
        hoodEncoder.setPosition(0.0);
        rotationEncoder.setPosition(0.0);
    }

    @Override // TODO: Not needed
    // Sets the brake mode
    public void setBrakeMode(boolean enabled) {
        // !Set flywheelMotor brake mode
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        IdleMode mode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
        // !Do flywheelMotor idleMode
        hoodConfig.idleMode(mode);
        rotationConfig.idleMode(mode);
        // !Apply flywheelMotor configuration
        hoodMotor.configure(
            hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotationMotor.configure(
            rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
