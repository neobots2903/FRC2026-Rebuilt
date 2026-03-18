package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class climb extends SubsystemBase {
  private final climbIO io;
  private final climbIOInputsAutoLogged inputs = new climbIOInputsAutoLogged();

  public enum climbRotation {
    IN(0.0),
    OUT(360.0);
    public final double angleDegrees;

    climbRotation(double angleDegrees) {
      this.angleDegrees = angleDegrees;
    }
  }

  public climb(climbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Logs pretty much everything
    Logger.processInputs("Climb/Climb", inputs);
    Logger.recordOutput("Climb/ClimbRotationDegrees", inputs.climbRotationDegrees);
    Logger.recordOutput("Climb/ClimbCurrentAmps", inputs.climbAppliedCurrentAmps);
    Logger.recordOutput("Climb/ClimbAppliedVolts", inputs.climbAppliedVolts);
  }

  // Controls the climb:

  // Sets the climb's rotation
  public void setClimbRotationDegrees(climbRotation rotation) {
    io.setClimbRotationDegrees(rotation.angleDegrees);
  }
  // Sets the climb's angle
  public void setClimbAngle(double angleDegrees) {
    double clamped = Math.max(climbConstants.kIn, Math.min(climbConstants.kOut, angleDegrees));
    io.setClimbRotationDegrees(clamped);
  }
  // Gets the climb's angle
  public double getClimbAngle() {
    return inputs.climbRotationDegrees;
  }
  // Determines the climb's rotation
  public boolean isClimbAtRotation(climbRotation rotation) {
    return Math.abs(inputs.climbRotationDegrees - rotation.angleDegrees)
        < climbConstants.kClimbPositionTolerance;
  }

  // Controls general elements:

  // Stops the system
  public void stop() {
    io.stop();
  }
  // Resets the encoder
  public void resetEncoders() {
    io.resetEncoders();
  }
  // Sets the brake mode
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }
}
