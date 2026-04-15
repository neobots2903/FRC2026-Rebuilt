package frc.robot.subsystems.Indexer

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.shooter.shooterConstants
import org.littletonrobotics.junction.Logger

class Indexer(private val io: IndexerIO) : SubsystemBase() {
    private val inputs = IndexerIOInputsAutoLogged()

    companion object {
        private val INDEXER_VOLTAGE = shooterConstants.kIndexerVoltage
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer/Indexer", inputs)
        Logger.recordOutput("Indexer/IndexerAppliedVolts", inputs.indexerAppliedVolts)
        Logger.recordOutput("Indexer/IndexerAppliedCurrentAmps", inputs.indexerAppliedCurrentAmps)
    }

    // Starts indexer
    fun startIndexer() {
        io.setIndexerVoltage(INDEXER_VOLTAGE)
    }

    // Stops indexer
    fun stopIndexer() {
        io.setIndexerVoltage(0.0)
    }

    // Determines if the indexer is running
    fun isIndexerRunning(): Boolean {
        return inputs.indexerAppliedVolts > 0.1
    }

    // Stops the systems
    fun stop() {
        io.stop()
    }
}
