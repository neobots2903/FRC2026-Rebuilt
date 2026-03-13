package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface indexerIO {
  @AutoLog
  public static class indexerIOInputs {
    public double indexerAppliedCurrentAmps = 0.0;
    public double indexerAppliedVolts = 0.0;
  }

  // Updates loggable inputs
  public default void updateInputs(indexerIOInputs inputs) {}
  // Sets the voltage for the indexer
  public default void setIndexerVoltage(double voltage) {}
  // Stops the indexer
  public default void stop() {}
}
