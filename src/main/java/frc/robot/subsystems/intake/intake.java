package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// The main body of the intake subsystem, which will go into the robot container script
public class intake extends SubsystemBase {
  private final intakeIO io;
  private final intakeIOInputsAutoLogged inputs = new intakeIOInputsAutoLogged();
  private static final double INTAKE_VOLTAGE = intakeConstants.kIntakeVoltage;
  // Counters
  public boolean intakeActive = false;
  public boolean pivotActive = false;

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
    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logs pretty much everything
    Logger.processInputs("Intake/Intake", inputs);
    Logger.recordOutput("Intake/PivotPositionDegrees", inputs.pivotPositionDegrees);
    Logger.recordOutput("Intake/PivotPositionDegrees", inputs.pivotPositionDegrees);
    Logger.recordOutput("Intake/PivotCurrentAmps", inputs.pivotAppliedCurrentAmps);
    Logger.recordOutput("Intake/PivotAppliedVolts", inputs.pivotAppliedVolts);
    Logger.recordOutput("Intake/IntakeCurrentAmps", inputs.intakeAppliedCurrentAmps);
    Logger.recordOutput("Intake/IntakeAppliedVolts", inputs.intakeAppliedVolts);
  }

  // Controls the intake:

  // Starts intake
  public void startIntake() {
    io.setIntakeVoltage(INTAKE_VOLTAGE);
  }
  // Stops intake
  public void stopIntake() {
    io.setIntakeVoltage(0);
  }
  // Determines if the intake is running
  public boolean isIntakeRunning() {
    return inputs.intakeAppliedCurrentAmps > 0.1;
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
