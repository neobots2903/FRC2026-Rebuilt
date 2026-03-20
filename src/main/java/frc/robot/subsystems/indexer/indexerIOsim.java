package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.indexer.indexerIO.indexerIOInputs;
import frc.robot.subsystems.shooter.shooterConstants;

public class indexerIOsim implements indexerIO {
  // The simulated motor
  private final DCMotorSim indexerMotorSim;
  // The applied voltage
  private double indexerAppliedVolts = 0.0;
  // The constants for the simulation
  private static final double MOI = 0.01;

  public indexerIOsim() {
    indexerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getVex775Pro(1), MOI, shooterConstants.kIndexerGearRatio),
            DCMotor.getVex775Pro(1));
  }

  @Override
  public void updateInputs(indexerIOInputs inputs) {
    // Updates the simulation
    indexerMotorSim.setInputVoltage(indexerAppliedVolts);
    indexerMotorSim.update(0.02);
    // Sets the inputs for the indexer
    inputs.indexerAppliedCurrentAmps = indexerMotorSim.getCurrentDrawAmps();
    inputs.indexerAppliedVolts = indexerAppliedVolts;
  }

  @Override
  // Sets the voltage for the indexer
  public void setIndexerVoltage(double voltage) {
    indexerAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    indexerMotorSim.setInputVoltage(voltage);
  }

  @Override
  // Stops the indexer
  public void stop() {
    indexerAppliedVolts = 0.0;
  }
}
