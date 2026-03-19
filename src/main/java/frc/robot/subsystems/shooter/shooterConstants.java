package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation3d;

public class shooterConstants {
  // Motor IDs
  public static final int kFlywheelMotorID = 40;
  public static final int kFlywheelMotor2ID = 41;
  public static final int kHoodMotorID = 32;
  public static final int kRotationMotorID = 14;
  public static final int kIndexerMotorID = 33;

  // Flywheel RPM
  public static final int ShooterRPM = 2000; // Placeholder
  // Angles (in degrees)
  public static final int kHoodAngleOffset = 20;
  public static final int kMinHoodAngle = 20;
  public static final int kMaxHoodAngle = 40;

  public static final double kFeedForward = 0.5; // Placeholder

  public static final int kCurrentLimit = 5; // Placeholder

  public static final double kGearRatio = 1.0;

  public static final double kHoodPositionTolerance = 2.0;
  public static final double kRotationPositionTolerance = 2.0;

  public static final double kTurretDegreesPerRotation = 1.0; // Placeholder

  public static final double kIndexerVoltage = 8.0;

  // Old
  // public static final double TARGET_X_METERS = 4.56;
  // public static final double TARGET_Y_METERS = 4.035;
  public static final double TARGET_Z_METERS = 1.83;
  // public static final Translation3d TARGET_POSE_METERS =
  //     new Translation3d(TARGET_X_METERS, TARGET_Y_METERS, TARGET_Z_METERS);

  // Mechanical Ratios
  public static final double kHoodGearRatio = 20.0;
  public static final double kHoodDegreesPerRotation = 360.0 / kHoodGearRatio;
  public static final double kFlywheelGearRatio = 1.0; // Placeholder
  public static final double kIndexerGearRatio = 12.0;

  // PID Values
  public static final double kHoodP = 0.2; // Placeholder
  public static final double kHoodI = 0.0; // Placeholder
  public static final double kHoodD = 0.0; // Placeholder
  public static final double kFlywheelP = 1.0; // Placeholder
  public static final double kFlywheelI = 0.1; // Placeholder
  public static final double kFlywheelD = 0.0; // Placeholder

  // Field-specific constants
  public static final double FIELD_LENGTH = 16.54; // meters
  public static final double FIELD_WIDTH = 8.21; // meters
  public static final double HUB_HEIGHT = 1.8288; // 72 inches in meters

  // Alliance-specific targets
  public static final Translation3d BLUE_HUB_POSITION =
      new Translation3d(4.5, FIELD_WIDTH / 2, TARGET_Z_METERS);
  public static final Translation3d RED_HUB_POSITION =
      new Translation3d(FIELD_LENGTH - 4.5, FIELD_WIDTH / 2, TARGET_Z_METERS);
  public static final Translation3d BLUE_DRIVER_STATION =
      new Translation3d(0.5, FIELD_WIDTH / 2, 0);
  public static final Translation3d RED_DRIVER_STATION =
      new Translation3d(FIELD_LENGTH - 0.5, FIELD_WIDTH / 2, 0);
}
