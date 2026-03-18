package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface climbIO {
  @AutoLog
  public static class climbIOInputs {
    // Things for figuring out control
    public boolean climbUp = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    // Motor inputs
    public double climbRotationDegrees = 0.0;
    public double climbAppliedCurrentAmps = 0.0;
    public double climbAppliedVolts = 0.0;
    public double climbSetRotationDegrees = 0.0;
  }

  // Updates loggable inputs
  public default void updateInputs(climbIOInputs inputs) {}
  // Sets the climb rotation (degrees)
  public default void setClimbRotationDegrees(double positionDegrees) {}
  // Stops all motors
  public default void stop() {}
  // Resets encoders back to zero
  public default void resetEncoders() {}
  // Decides whether or not brake mode is enabled
  public default void setBrakeMode(boolean enabled) {}
}
