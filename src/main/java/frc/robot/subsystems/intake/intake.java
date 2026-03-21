package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// The main body of the intake subsystem, which will go into the robot container script
public class intake extends SubsystemBase {
  private final intakeIO io;
  private final intakeIOInputsAutoLogged inputs = new intakeIOInputsAutoLogged();
  private static final double INTAKE_VOLTAGE = intakeConstants.kIntakeVoltage;

  public enum pivotPosition {
    IN(0.0),
    OUT(120.0);
    public final double angleDegrees;

    pivotPosition(double angleDegrees) {
      this.angleDegrees = angleDegrees;
    }
  }

  public intake(intakeIO io) {
    this.io = io;
    // io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logs pretty much everything
    Logger.processInputs("Intake/Intake", inputs);
    Logger.recordOutput("Intake/PivotPositionDegrees", inputs.pivotPositionDegrees);
    Logger.recordOutput("Intake/PivotCurrentAmps", inputs.pivotAppliedCurrentAmps);
    Logger.recordOutput("Intake/PivotAppliedVolts", inputs.pivotAppliedVolts);
    Logger.recordOutput("Intake/IntakeCurrentAmps", inputs.intakeAppliedCurrentAmps);
    Logger.recordOutput("Intake/IntakeAppliedVolts", inputs.intakeAppliedVolts);
    Logger.recordOutput("Intake/IntakeRPM", inputs.intakeRPM);
    Logger.recordOutput("Intake/SafeToIntake", isSafeToIntake());
    Logger.recordOutput("Intake/IsStowed", isStowed());
  }

  // Controls the intake:

  // Check if the intake pivot is deployed enough to safely run the intake wheels
  public boolean isSafeToIntake() {
    return getPivotAngle() > 50;
  }

  // Starts intake to shoot out the back (only when safe)
  public void shootOutBack() {
    if (isSafeToIntake()) {
      io.setIntakeVoltage(12);
    } else {
      io.setIntakeVoltage(0); // Ensure stopped when not safe
    }
  }

  // Starts intake wheels (only when safe)
  public void startIntake() {
    if (isSafeToIntake()) {
      io.setIntakeVoltage(5);
    } else {
      io.setIntakeVoltage(0); // Ensure stopped when not safe
    }
  }

  // Stops intake
  public void stopIntake() {
    io.setIntakeVoltage(0);
  }

  // Determines if the intake is running
  public boolean isIntakeRunning() {
    return inputs.intakeAppliedVolts > 0.1;
  }

  // Check if the intake is stowed (retracted)
  public boolean isStowed() {
    return getPivotAngle() < intakeConstants.kPivotPositionTolerance;
  }

  // Controls the pivot:

  // Sets the pivot's position
  public void setPivotPosition(pivotPosition position) {
    io.setPivotPosition(position.angleDegrees);
  }
  // Sets the pivot's angle
  public void setPivotAngle(double angleDegrees) {
    double clamped =
        Math.max(intakeConstants.kInPosition, Math.min(intakeConstants.kOutPosition, angleDegrees));
    io.setPivotPosition(clamped);
  }
  // Gets the pivot's angle
  public double getPivotAngle() {
    return inputs.pivotPositionDegrees;
  }
  // Determines the pivot's position
  public boolean isPivotAtPosition(pivotPosition position) {
    return Math.abs(inputs.pivotPositionDegrees - position.angleDegrees)
        < intakeConstants.kPivotPositionTolerance;
  }

  // Controls general elements:

  // Stops the systems
  public void stop() {
    io.stop();
  }
  // Resets the encoders
  public void resetEncoders() {
    io.resetEncoders();
  }
  // Sets the brake mode
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }
}
