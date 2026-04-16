package frc.robot.subsystems.Indexer

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.shooter.shooterConstants
import org.littletonrobotics.junction.Logger

class Indexer(private val io: IndexerIO) : SubsystemBase() {
    private val inputs = IndexerIOInputsAutoLogged()

    // Constants
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

/*
Notes:

- 04/15/26
Kotlin is a nicer language to write, and it is safer by default than Java. That being said, it seems like the VSCode support
for Kotlin is not good. Some teams use IntelliJ, which has more support for Kotlin but less for FRC. So far, the pros do not outweigh the cons.

*/

/* Try out OBJECT with factory pattern once more Kotlin is in use. a little cleaner since subsystems can ONLY have one instance. */

// package frc.robot.subsystems.Indexer

// import edu.wpi.first.wpilibj2.command.SubsystemBase
// import frc.robot.subsystems.shooter.shooterConstants
// import org.littletonrobotics.junction.Logger

// object Indexer : SubsystemBase() {
//     private lateinit var io: IndexerIO
//     private lateinit var inputs: IndexerIOInputsAutoLogged

//     private val INDEXER_VOLTAGE = shooterConstants.kIndexerVoltage

//     // Factory-style initializer — call once from RobotContainer
//     fun create(io: IndexerIO): Indexer {
//         this.io = io
//         this.inputs = IndexerIOInputsAutoLogged()
//         return this
//     }

//     override fun periodic() {
//         io.updateInputs(inputs)
//         Logger.processInputs("Indexer/Indexer", inputs)
//         Logger.recordOutput("Indexer/IndexerAppliedVolts", inputs.indexerAppliedVolts)
//         Logger.recordOutput("Indexer/IndexerAppliedCurrentAmps", inputs.indexerAppliedCurrentAmps)
//     }

//     fun startIndexer() {
//         io.setIndexerVoltage(INDEXER_VOLTAGE)
//     }

//     fun stopIndexer() {
//         io.setIndexerVoltage(0.0)
//     }

//     fun isIndexerRunning(): Boolean {
//         return inputs.indexerAppliedVolts > 0.1
//     }

//     fun stop() {
//         io.stop()
//     }
// }
