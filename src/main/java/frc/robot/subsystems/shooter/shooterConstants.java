package frc.robot.subsystems.shooter;

public class shooterConstants {
  // Motor IDs
  public static final int kFlywheelMotorID = 11;
  public static final int kHoodMotorID = 12;
  public static final int kRotationMotorID = 13;
  // Flywheel RPM
  public static final int ShooterRPM = 1; // Placeholder
  // Angles (in degrees)
  public static final int kMinHoodAngle = 0; // Placeholder
  public static final int kMaxHoodAngle = 0; // Placeholder
  public static final int kMinRotationAngle = 0; // Placeholder
  public static final int kMaxRotationAngle = 0; // Placeholder

  public static final double kFeedForward = 0.5; // Placeholder

  public static final int kCurrentLimit = 40; // Placeholder

  public static final double kGearRatio = 1.0;

  public static final double kHoodPositionTolerance = 2.0;
  public static final double kRotationPositionTolerance = 2.0;

  public static final double kTurretDegreesPerRotation = 1.0; // Placeholder

  // Mechanical Ratios
  public static final double kHoodDegreesPerRotation = 360.0 / 1.0; // Placeholder
  public static final double kFlywheelGearRatio = 1.0; // Placeholder

  // PID Values
  public static final double kHoodP = 1.0; // Placeholder
  public static final double kHoodI = 0.1; // Placeholder
  public static final double kHoodD = 0.0; // Placeholder
  public static final double kFlywheelP = 1.0; // Placeholder
  public static final double kFlywheelI = 0.1; // Placeholder
  public static final double kFlywheelD = 0.0; // Placeholder

  // Peak Kraken Power = 3000 RPM
}
