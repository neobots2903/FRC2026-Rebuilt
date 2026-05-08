package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooterConstants;
import org.littletonrobotics.junction.Logger;

public class indexer extends SubsystemBase {
  private final indexerIO io;
  private final indexerIOInputsAutoLogged inputs = new indexerIOInputsAutoLogged();
  private static final double INDEXER_VOLTAGE = shooterConstants.kIndexerVoltage;

  public indexer(indexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logs pretty much everything
    Logger.processInputs("Indexer/Indexer", inputs);
    Logger.recordOutput("Indexer/IndexerAppliedVolts", inputs.indexerAppliedVolts);
    Logger.recordOutput("Indexer/IndexerAppliedCurrentAmps", inputs.indexerAppliedCurrentAmps);
  }

  // Controls the indexer:

  // Starts indexer
  public void startIndexer() {
    io.setIndexerVoltage(INDEXER_VOLTAGE);
  }
  // Stops indexer
  public void stopIndexer() {
    io.setIndexerVoltage(0);
  }
  // Determines if the indexer is running
  public boolean isIndexerRunning() {
    return inputs.indexerAppliedVolts > 0.1;
  }

  // Controls general elements:

  // Stops the systems
  public void stop() {
    io.stop();
  }
}
