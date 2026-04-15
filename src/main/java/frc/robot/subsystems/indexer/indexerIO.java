package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double indexerAppliedCurrentAmps = 0.0;
    public double indexerAppliedVolts = 0.0;
  }

  // Updates loggable inputs
  public default void updateInputs(IndexerIOInputs inputs) {}
  // Sets the voltage for the indexer
  public default void setIndexerVoltage(double voltage) {}
  // Stops the indexer
  public default void stop() {}
}
