package frc.robot.subsystems.shooter;

public class shooterConstants {
  // Motor IDs
  public static final int kFlywheelMotorID = 40;
  public static final int kFlywheelMotor2ID = 41;
  public static final int kHoodMotorID = 32;
  public static final int kIndexerMotorID = 33;

  // Default flywheel RPM (used by manual startFlywheel())
  public static final int ShooterRPM = 2000;

  // Hood angle offset: encoder reads 0 when physical angle is 20°
  public static final int kHoodAngleOffset = 20;
  // Min/max are in PHYSICAL hood angle degrees
  public static final int kMinHoodAngle = 21; // encoder = 1°
  public static final int kMaxHoodAngle = 35; // encoder = 15°

  // Current limits
  public static final int kFlywheelCurrentLimit = 60;
  public static final int kHoodCurrentLimit = 10;
  public static final int kIndexerCurrentLimit = 7;

  // Sim gear ratio (flywheel is direct-drive)
  public static final double kGearRatio = 1.0;

  // Tolerances
  public static final double kHoodPositionTolerance = 2.0;

  // Indexer
  public static final double kIndexerVoltage = 8.0;

  // Target height (hub rim, meters)
  public static final double TARGET_Z_METERS = 1.83;

  // Mechanical ratios
  public static final double kHoodGearRatio = 20.0;
  public static final double kHoodDegreesPerRotation = 360.0 / kHoodGearRatio;
  public static final double kFlywheelGearRatio = 1.0;
  public static final double kIndexerGearRatio = 12.0;

  // PID values
  public static final double kHoodP = 0.05;
  public static final double kHoodI = 0.0;
  public static final double kHoodD = 0.0;

  public static final double kFlywheelP = 0.5;
  public static final double kFlywheelI = 0.0;
  public static final double kFlywheelD = 0.0;
  public static final double kFeedForward = 4.0;

  // Field dimensions
  public static final double FIELD_LENGTH = 16.54; // meters
  public static final double FIELD_WIDTH = 8.21; // meters
}
