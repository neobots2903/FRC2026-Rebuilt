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
  private final DCMotorSim flywheelMotor2Sim;
  private final DCMotorSim hoodMotorSim;
  // PID controllers
  private final PIDController hoodPID;
  private final PIDController flywheelPID;

  private Double hoodSetpointDegrees = null;
  private Double flywheelSetpointRPM = null;
  private double flywheelAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;
  private double flywheelVelocity = 0.0;
  private Double flywheel2SetpointRPM = null;
  private double flywheel2AppliedVolts = 0.0;
  private double flywheel2Velocity = 0.0;
  // Constants for the simulation
  private static final double MOI = 0.01;

  public shooterIOsim() {
    flywheelMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), MOI, shooterConstants.kGearRatio),
            DCMotor.getKrakenX60(1));
    flywheelPID = new PIDController(0.06, 0.0, 0.0);
    flywheelMotor2Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), MOI, shooterConstants.kGearRatio),
            DCMotor.getKrakenX60(1));
    hoodMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1), MOI, shooterConstants.kGearRatio),
            DCMotor.getNeo550(1));
    hoodPID = new PIDController(0.03, 0.0, 0.01);
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
    flywheelMotorSim.update(0.02);
    flywheelMotor2Sim.update(0.02);
    hoodMotorSim.setInputVoltage(hoodAppliedVolts);
    hoodMotorSim.update(0.02);
    // Inputs
    inputs.flywheelCurrent = flywheelMotorSim.getCurrentDrawAmps();
    inputs.flywheelVelocity = getFlywheelRPM();
    inputs.flywheelAppliedVolts = flywheelAppliedVolts;
    inputs.flywheel2Current = flywheelMotor2Sim.getCurrentDrawAmps();
    inputs.flywheel2Velocity = getFlywheelRPM();
    inputs.flywheel2AppliedVolts = flywheel2AppliedVolts;
    inputs.hoodPositionDegrees =
        hoodMotorSim.getAngularPositionRotations() * 360.0 + shooterConstants.kHoodAngleOffset;
    inputs.hoodAppliedCurrentAmps = hoodMotorSim.getCurrentDrawAmps();
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodSetPointDegrees =
        (hoodSetpointDegrees != null) ? hoodSetpointDegrees : inputs.hoodPositionDegrees;

    if (flywheelSetpointRPM != null) {
      double currentRPM = getFlywheelRPM();
      double pidOutput = flywheelPID.calculate(currentRPM, flywheelSetpointRPM);
      flywheelAppliedVolts = MathUtil.clamp(pidOutput, -12.0, 12.0);
      flywheelMotorSim.setInputVoltage(flywheelAppliedVolts);
      inputs.flywheelSetpointRPM = flywheelSetpointRPM;
    } else {
      inputs.flywheelSetpointRPM = 0.0;
    }

    if (flywheel2SetpointRPM != null) {
      double currentRPM2 = getFlywheelRPM();
      double pidOutput2 = flywheelPID.calculate(currentRPM2, flywheel2SetpointRPM);
      flywheel2AppliedVolts = MathUtil.clamp(pidOutput2, -12.0, 12.0);
      flywheelMotor2Sim.setInputVoltage(flywheel2AppliedVolts);
      inputs.flywheel2SetpointRPM = flywheel2SetpointRPM;
    } else {
      inputs.flywheel2SetpointRPM = 0.0;
    }
  }

  @Override
  // Sets the velocity for the flywheel
  public void setFlywheelVelocity(double velocityRPM) {
    flywheelSetpointRPM = velocityRPM;
    flywheel2SetpointRPM = velocityRPM;
  }

  private double getFlywheelRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(
        flywheelMotorSim.getAngularVelocityRadPerSec());
  }

  private double getFlywheel2RPM() {
    return Units.radiansPerSecondToRotationsPerMinute(
        flywheelMotor2Sim.getAngularVelocityRadPerSec());
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
    flywheel2AppliedVolts = 0.0;
    hoodAppliedVolts = 0.0;
    flywheelMotorSim.setInputVoltage(flywheelAppliedVolts);
    flywheelMotor2Sim.setInputVoltage(flywheel2AppliedVolts);
    hoodMotorSim.setInputVoltage(hoodAppliedVolts);
    flywheelSetpointRPM = null;
  }

  @Override
  // Resets encoders
  public void resetEncoders() {
    hoodMotorSim.setState(0.0, 0.0);
  }
}
