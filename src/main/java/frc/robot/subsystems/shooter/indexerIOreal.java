package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class indexerIOreal implements indexerIO {

  // Indexer motor
  private final SparkMax indexerMotor;

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
    config.smartCurrentLimit(shooterConstants.kIndexerCurrentLimit);
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
}
