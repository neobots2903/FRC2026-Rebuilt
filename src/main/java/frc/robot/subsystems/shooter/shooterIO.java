package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface shooterIO {
  @AutoLog
  public static class shooterIOInputs {
    // flywheelMotor inputs

    // hoodMotor inputs
    public double hoodPositionDegrees = 0.0;
    public double hoodAppliedCurrentAmps = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodSetPointDegrees = 0.0;

    public double flywheelVelocity = 0.0;
    public double flywheelCurrent = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelSetpointRPM = 0.0;
    public double flywheel2Velocity = 0.0;
    public double flywheel2Current = 0.0;
    public double flywheel2AppliedVolts = 0.0;
    public double flywheel2SetpointRPM = 0.0;
  }

  public default void setFlywheelVelocity(double velocityRPM) {}

  public default void updateInputs(shooterIOInputs inputs) {}

  public default void setHoodPosition(double positionDegrees) {}

  public default void stop() {}

  public default void resetEncoders() {}
}
