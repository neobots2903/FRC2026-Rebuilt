package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

public class indexerIOreal implements indexerIO{

    // Indexer motor
    private final SparkMax indexerMotor;

    private final RelativeEncoder indexerEncoder;
    private final SparkClosedLoopController indexerPID;

    // Generally sets up the motor
    @SuppressWarnings("removal")
    public indexerIOreal() {
        indexerMotor = new SparkMax(shooterConstants.kIndexerMotorID, MotorType.kBrushless);
        indexerMotor.configure(
            configureIndexerMotor(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Configures the indexer motor
    private SparkMaxConfig configureIndexerMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(shooterConstants.kCurrentLimit);
        config.voltageCompensation(12.0);
        return config;
    }

    @Override
    // Updates inputs
    public void updateInputs(indexerIOInputs inputs) {
        // Updates the indexer inputs
        inputs.indexerAppliedCurrentAmps = indexerMotor.getOutputCurrent();
        inputs.indexerAppliedVolts = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
    }

    @Override
    // Sets the voltage for the indexer
    public void setIndexerVoltage(double voltage) {
        double clampedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        indexerMotor.setVoltage(clampedVolts);
    }

    @Override
    // Stops the indexer
    public void stop() {
        indexerMotor.stopMotor();
    }

    @Override
    // Sets the brake mode
    public void setBrakeMode(boolean enabled) {
        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        IdleMode mode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
        indexerConfig.idleMode(mode);
        indexerMotor.configure(
            indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}


