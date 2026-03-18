package frc.robot.subsystems.climb;

public class climbConstants {
  // Motor ID
  public static final int kClimbMotorId = 42;
  // Positions (in degrees)
  public static final double kIn = 0.0;
  public static final double kOut = 360.0;
  // Current limit
  public static final int kClimbCurrentLimit = 40;
  // PID values
  public static final double kClimbP = 0.1;
  public static final double kClimbI = 0.0;
  public static final double kClimbD = 0.0;
  // Tolerance
  public static final double kClimbPositionTolerance = 2.0; // In degrees
  // Mechanical configurations
  public static final double kClimbGearRatio = 1.0; // Placeholder
  // Conversion factors
  public static final double kClimbDegreesPerRotation = 360.0 / kClimbGearRatio;
}
