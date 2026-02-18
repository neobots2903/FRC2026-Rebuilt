package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class intakeIOsim implements intakeIO {
  // The simulated motors
  private final DCMotorSim pivotMotorSim;
  private final DCMotorSim intakeMotorSim;
  // The PID controller for the pivot position
  private final PIDController pivotPID;
  // The tracking for the control mode
  private Double pivotSetpointDegrees = null;
  // The applied voltages
  private double pivotAppliedVolts = 0.0;
  private double intakeAppliedVolts = 0.0;
  // The constants for the simulation
  private static final double MOI = 0.01;

  public intakeIOsim() {
    pivotMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getBag(1), MOI, intakeConstants.kPivotGearRatio),
            DCMotor.getBag(1));
    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getVex775Pro(1), MOI, intakeConstants.kIntakeGearRatio),
            DCMotor.getCIM(1));
    pivotPID =
        new PIDController(
            intakeConstants.kPivotP, intakeConstants.kPivotI, intakeConstants.kPivotD);
    pivotPID.setTolerance(intakeConstants.kPivotPositionTolerance);
  }

  @Override
  // Updates the pivot control
  public void updateInputs(intakeIOInputs inputs) {
    if (pivotSetpointDegrees != null) {
      double pivotOutput = pivotPID.calculate(inputs.pivotPositionDegrees, pivotSetpointDegrees);
      pivotAppliedVolts = Math.max(-12.0, Math.min(12.0, pivotOutput));
    } else {
      pivotAppliedVolts = 0.0;
    }
    // Updates the simulations
    pivotMotorSim.setInputVoltage(pivotAppliedVolts);
    pivotMotorSim.update(0.02);
    intakeMotorSim.setInputVoltage(intakeAppliedVolts);
    intakeMotorSim.update(0.02);
    // Sets the inputs for the pivot
    inputs.pivotPositionDegrees = pivotMotorSim.getAngularPositionRotations() * 360.0;
    inputs.pivotAppliedCurrentAmps = pivotMotorSim.getCurrentDrawAmps();
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotSetPointDegrees =
        (pivotSetpointDegrees != null) ? pivotSetpointDegrees : inputs.pivotPositionDegrees;
    // Sets the inputs for the intake
    inputs.intakeAppliedCurrentAmps = intakeMotorSim.getCurrentDrawAmps();
    inputs.intakeAppliedVolts = intakeAppliedVolts;
  }

  @Override
  // Sets the position for the pivot
  public void setPivotPosition(double angleDegrees) {
    pivotSetpointDegrees =
        MathUtil.clamp(angleDegrees, intakeConstants.kInPosition, intakeConstants.kOutPosition);
  }

  @Override
  // Sets the voltage for the intake
  public void setIntakeVoltage(double voltage) {
    intakeAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    intakeMotorSim.setInputVoltage(voltage);
  }

  @Override
  // Stops the systems
  public void stop() {
    pivotSetpointDegrees = null;
    pivotAppliedVolts = 0.0;
    setIntakeVoltage(0);
    pivotMotorSim.setInputVoltage(pivotAppliedVolts);
  }

  @Override
  // Resets encoders
  public void resetEncoders() {
    pivotMotorSim.setState(0.0, 0.0);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // Simulated brake mode; handled automatically in the simulation
  }
}
