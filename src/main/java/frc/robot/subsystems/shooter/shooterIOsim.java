package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.intakeIO.intakeIOInputs;

public class shooterIOsim implements shooterIO{
    // Simulated motors
    // !flywheel Motor
    private final DCMotorSim flywheelMotorSim;
    private final DCMotorSim hoodMotorSim;
    private final DCMotorSim rotationMotorSim;
    // PID controllers
    private final PIDController hoodPID;
    private final PIDController rotationPID;

    private Double hoodSetpointDegrees = null;
    private Double rotationSetpointDegrees = null;
    private double flywheelAppliedVolts = 0.0;
    private double hoodAppliedVolts = 0.0;
    private double rotationAppliedVolts = 0.0;
    // Constants for the simulation
    private static final double MOI = 0.01;

    public shooterIOsim() {
        flywheelMotorSim = 
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1), MOI, shooterConstants.kGearRatio),
                DCMotor.getKrakenX60(1));
        hoodMotorSim = 
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getNeo550(1), MOI, shooterConstants.kGearRatio),
                DCMotor.getNeo550(1));
        rotationMotorSim = 
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getNEO(1), MOI, shooterConstants.kGearRatio),
                DCMotor.getNEO(1));
        hoodPID = 
            new PIDController(
                shooterConstants.kHoodP, shooterConstants.kHoodI, shooterConstants.kHoodD);
        hoodPID.setTolerance(shooterConstants.kHoodPositionTolerance);
        rotationPID = 
            new PIDController(
                shooterConstants.kRotationP, shooterConstants.kRotationI, shooterConstants.kRotationD);
        rotationPID.setTolerance(shooterConstants.kRotationPositionTolerance);
    }

    @Override
    // Updates inputs
    public void updateInputs(intakeIOInputs inputs) {
        if (hoodSetpointDegrees != null) {
            double hoodOutput = hoodPID.calculate(inputs.hoodPositionDegrees, hoodSetpointDegrees);
            hoodAppliedVolts = Math.max(-12.0, Math.min(12.0, hoodOutput));
        } else {
            hoodAppliedVolts = 0.0;
        }
        if (rotationSetpointDegrees != null) {
            double rotationOutput = rotationPID.calculate(inputs.rotationPositionDegrees, rotationSetpointDegrees);
            rotationAppliedVolts = Math.max(-12.0, Math.min(12.0, rotationOutput));
        } else {
            rotationAppliedVolts = 0.0;
        }
        // Updates
        flywheelMotorSim.setInputVoltage(flywheelAppliedVolts);
        flywheelMotorSim.update(0.02);
        hoodMotorSim.setInputVoltage(hoodAppliedVolts);
        hoodMotorSim.update(0.02);
        rotationMotorSim.setInputVoltage(rotationAppliedVolts);
        rotationMotorSim.update(0.02);
        // Inputs
        inputs.flywheelAppliedCurrentAmps = flywheelMotorSim.getCurrentDrawAmps();
        inputs.flywheelAppliedVolts = flywheelAppliedVolts;
        inputs.hoodPositionDegrees = hoodMotorSim.getAngularPositionRotations() * 360.0;
        inputs.hoodAppliedCurrentAmps = hoodMotorSim.getCurrentDrawAmps();
        inputs.hoodAppliedVolts = hoodAppliedVolts;
        inputs.hoodSetPointDegrees = 
            (hoodSetpointDegrees != null) ? hoodSetpointDegrees : inputs.hoodPositionDegrees;
        inputs.rotationPositionDegrees = rotationMotorSim.getAngularPositionRotations() * 360.0;
        inputs.rotationAppliedCurrentAmps = rotationMotorSim.getCurrentDrawAmps();
        inputs.rotationAppliedVolts = rotationAppliedVolts;
        inputs.rotationSetPointDegrees = 
            (rotationSetpointDegrees != null) ? rotationSetpointDegrees : inputs.rotationPositionDegrees;
    }

    @Override
    // Sets the angle for the hood
    public void setHoodPosition(double angleDegrees) {
        hoodSetpointDegrees = 
            MathUtil.clamp(angleDegrees, shooterConstants.kMinHoodAngle, shooterConstants.kMaxHoodAngle);
    }

    @Override
    // Sets the angle for the rotation
    public void setRotationPosition(double angleDegrees) {
        rotationSetpointDegrees = 
            MathUtil.clamp(angleDegrees, shooterConstants.kMinRotationAngle, shooterConstants.kMaxRotationAngle);
    }

    @Override
    // Sets flywheel voltage
    public void setFlywheelVoltage(double voltage) {
        flywheelAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        flywheelMotorSim.setFlywheelVoltage(voltage);
    }

    @Override
    // Stops the systems
    public void stop () {
        hoodSetpointDegrees = null;
        rotationSetpointDegrees = null;
        flywheelAppliedVolts = 0.0;
        hoodAppliedVolts = 0.0;
        rotationAppliedVolts = 0.0;
        flywheelMotorSim.setInputVoltage(flywheelAppliedVolts);
        hoodMotorSim.setInputVoltage(hoodAppliedVolts);
        rotationMotorSim.setInputVoltage(rotationAppliedVolts);
    }

    @Override
    // Resets encoders
    public void resetEncoders() {
        hoodMotorSim.setState(0.0, 0.0);
        rotationMotorSim.setState(0.0, 0.0);
    }

    @Override
    // Sets brake mode
    public void setBrakeMode(boolean enabled) {
        // Handled automatically in the simulation
    }
}
