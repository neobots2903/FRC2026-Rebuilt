package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.climb.climbIO.climbIOInputs;

public class climbIOsim implements climbIO {
  // The simulated motor
  private final DCMotorSim climbMotorSim;
  // The PID controller
  private final PIDController climbPID;
  // The tracking for the control mode
  private Double climbSetpointDegrees = null;
  // Applied voltage
  private double climbAppliedVolts = 0.0;
  // Constants for the simulation
  private static final double MOI = 0.01;

  public climbIOsim() {
    climbMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), MOI, climbConstants.kClimbGearRatio),
            DCMotor.getNEO(1));
    climbPID =
        new PIDController(climbConstants.kClimbP, climbConstants.kClimbI, climbConstants.kClimbD);
    climbPID.setTolerance(climbConstants.kClimbPositionTolerance);
  }

  @Override
  // Updates the climb control
  public void updateInputs(climbIOInputs inputs) {
    if (climbSetpointDegrees != null) {
      double climbOutput = climbPID.calculate(inputs.climbRotationDegrees, climbSetpointDegrees);
      climbAppliedVolts = Math.max(-12.0, Math.min(12.0, climbOutput));
    } else {
      climbAppliedVolts = 0.0;
    }
    // Updates the simulations
    climbMotorSim.setInputVoltage(climbAppliedVolts);
    climbMotorSim.update(0.02);
    // Sets the inputs for the climb
    inputs.climbRotationDegrees = climbMotorSim.getAngularPositionRotations() * 360.0;
    inputs.climbAppliedCurrentAmps = climbMotorSim.getCurrentDrawAmps();
    inputs.climbAppliedVolts = climbAppliedVolts;
    inputs.climbSetRotationDegrees =
        (climbSetpointDegrees != null) ? climbSetpointDegrees : inputs.climbRotationDegrees;
  }

  @Override
  // Sets the rotation for the climb
  public void setClimbRotationDegrees(double angleDegrees) {
    climbSetpointDegrees = MathUtil.clamp(angleDegrees, climbConstants.kIn, climbConstants.kOut);
  }

  @Override
  // Stops the system
  public void stop() {
    climbSetpointDegrees = null;
    climbAppliedVolts = 0.0;
    climbMotorSim.setInputVoltage(climbAppliedVolts);
  }

  @Override
  // Resets encoders
  public void resetEncoders() {
    climbMotorSim.setState(0.0, 0.0);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // Simulated brake mode; handled automatically in the simulation
  }
}
