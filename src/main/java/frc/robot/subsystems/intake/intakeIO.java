package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface intakeIO {
  @AutoLog
  public static class intakeIOInputs {
    // Things for figuring out control
    public boolean intakeOn = false;
    public boolean pivotDown = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double turnVelocityDegPerSec = 0.0;
    // Pivot motor inputs
    public double pivotPositionDegrees = 0.0;
    public double pivotAppliedCurrentAmps = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSetPointDegrees = 0.0;
    // Intake motor inputs
    public double intakeAppliedCurrentAmps = 0.0;
    public double intakeAppliedVolts = 0.0;
  }

  // Updates loggable inputs
  public default void updateInputs(intakeIOInputs inputs) {}
  // Sets the pivot position (degrees)
  public default void setPivotPosition(double positionDegrees) {}
  // Sets the intake motor voltage directly
  public default void setIntakeVoltage(double voltage) {}
  // Stops all motors
  public default void stop() {}
  // Resets encoders back to zero
  public default void resetEncoders() {}
  // Decides whether or not brake mode is enabled
  public default void setBrakeMode(boolean enabled) {}
  // Sets the velocity
  public default void setVelocity(double intakeVelocity) {}
  // Sets the position
  public default void setPosition(double pivotPosition) {}
}
