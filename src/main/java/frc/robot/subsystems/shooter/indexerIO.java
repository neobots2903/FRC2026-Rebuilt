package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public class indexerIO {
    @AutoLog
    public static class indexerIOInputs {
        public double indexerAppliedCurrentAmps = 0.0;
        public double indexerAppliedVolts = 0.0;
    }

    // Updates loggable inputs
    public void updateInputs(indexerIOInputs inputs) {}
    // Sets the voltage for the indexer
    public void setIndexerVoltage(double voltage) {}
    // Stops the indexer
    public void stop() {}
    // Sets the brake mode
    public void setBrakeMode(boolean enabled) {}
}
