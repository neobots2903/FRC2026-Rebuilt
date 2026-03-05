package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class shooterIOsim implements shooterIO {
  // Simulated motors
  // !flywheel Motor
  private final DCMotorSim flywheelMotorSim;
  private final DCMotorSim hoodMotorSim;
  // PID controllers
  private final PIDController hoodPID;
  private final PIDController flywheelPID;

  private Double hoodSetpointDegrees = null;
  private Double flywheelSetpointRPM = null;
  private double flywheelAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;
  private double flywheelVelocity = 0.0;
  // Constants for the simulation
  private static final double MOI = 0.01;

  public shooterIOsim() {
    flywheelMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), MOI, shooterConstants.kGearRatio),
            DCMotor.getKrakenX60(1));
    flywheelPID =
        new PIDController(
            shooterConstants.kFlywheelP, shooterConstants.kFlywheelI, shooterConstants.kFlywheelD);
    hoodMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1), MOI, shooterConstants.kGearRatio),
            DCMotor.getNeo550(1));
    hoodPID =
        new PIDController(
            shooterConstants.kHoodP, shooterConstants.kHoodI, shooterConstants.kHoodD);
    hoodPID.setTolerance(shooterConstants.kHoodPositionTolerance);
  }

  @Override
  // Updates inputs
  public void updateInputs(shooterIOInputs inputs) {
    if (hoodSetpointDegrees != null) {
      double hoodOutput = hoodPID.calculate(inputs.hoodPositionDegrees, hoodSetpointDegrees);
      hoodAppliedVolts = Math.max(-12.0, Math.min(12.0, hoodOutput));
    } else {
      hoodAppliedVolts = 0.0;
    }
    // Updates
    flywheelMotorSim.setInputVoltage(flywheelAppliedVolts);
    flywheelMotorSim.update(0.02);
    hoodMotorSim.setInputVoltage(hoodAppliedVolts);
    hoodMotorSim.update(0.02);
    // Inputs
    inputs.flywheelCurrent = flywheelMotorSim.getCurrentDrawAmps();
    inputs.hoodPositionDegrees = hoodMotorSim.getAngularPositionRotations() * 360.0;
    inputs.hoodAppliedCurrentAmps = hoodMotorSim.getCurrentDrawAmps();
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodSetPointDegrees =
        (hoodSetpointDegrees != null) ? hoodSetpointDegrees : inputs.hoodPositionDegrees;

    if (flywheelSetpointRPM != null) {
      double currentRPM = getFlywheelRPM();
      double pidOutput = flywheelPID.calculate(currentRPM, flywheelSetpointRPM);
      flywheelAppliedVolts = MathUtil.clamp(pidOutput, -12.0, 12.0);
      flywheelMotorSim.setInputVoltage(flywheelAppliedVolts);
    }
  }

  @Override
  // Sets the velocity for the flywheel
  public void setFlywheelVelocity(double velocityRPM) {
    flywheelSetpointRPM = velocityRPM;
  }

  private double getFlywheelRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(
        flywheelMotorSim.getAngularVelocityRadPerSec());
  }

  @Override
  // Sets the angle for the hood
  public void setHoodPosition(double angleDegrees) {
    hoodSetpointDegrees =
        MathUtil.clamp(
            angleDegrees, shooterConstants.kMinHoodAngle, shooterConstants.kMaxHoodAngle);
  }

  @Override
  // Stops the systems
  public void stop() {
    hoodSetpointDegrees = null;
    flywheelAppliedVolts = 0.0;
    hoodAppliedVolts = 0.0;
    flywheelMotorSim.setInputVoltage(flywheelAppliedVolts);
    hoodMotorSim.setInputVoltage(hoodAppliedVolts);
  }

  @Override
  // Resets encoders
  public void resetEncoders() {
    hoodMotorSim.setState(0.0, 0.0);
  }

  @Override
  // Sets brake mode
  public void setBrakeMode(boolean enabled) {
    // Handled automatically in the simulation
  }
}
