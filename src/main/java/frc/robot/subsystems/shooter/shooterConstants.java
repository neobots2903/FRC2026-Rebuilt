package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class shooterConstants {
  // Motor IDs
  public static final int kFlywheelMotorID = 11;
  public static final int kHoodMotorID = 12;
  public static final int kRotationMotorID = 13;
  // Flywheel RPM
  public static final int ShooterRPM = 3000; // Placeholder
  // Angles (in degrees)
  public static final int kMinHoodAngle = 0; // Placeholder
  public static final int kMaxHoodAngle = 100; // Placeholder
  public static final int kMinRotationAngle = 0; // Placeholder
  public static final int kMaxRotationAngle = 0; // Placeholder

  public static final double kFeedForward = 0.5; // Placeholder

  public static final int kCurrentLimit = 40; // Placeholder

  public static final double kGearRatio = 1.0;

  public static final double kHoodPositionTolerance = 2.0;
  public static final double kRotationPositionTolerance = 2.0;

  public static final double kTurretDegreesPerRotation = 1.0; // Placeholder

  // Old
  // public static final double TARGET_X_METERS = 4.56;
  // public static final double TARGET_Y_METERS = 4.035;
  // public static final double TARGET_Z_METERS = 1.83;
  // public static final Translation3d TARGET_POSE_METERS =
  //     new Translation3d(TARGET_X_METERS, TARGET_Y_METERS, TARGET_Z_METERS);

  // Mechanical Ratios
  public static final double kHoodGearRatio = 4.0;
  public static final double kHoodDegreesPerRotation = 360.0 / kHoodGearRatio;
  public static final double kFlywheelGearRatio = 1.0; // Placeholder

  // PID Values
  public static final double kHoodP = 1.0; // Placeholder
  public static final double kHoodI = 0.1; // Placeholder
  public static final double kHoodD = 0.0; // Placeholder
  public static final double kFlywheelP = 1.0; // Placeholder
  public static final double kFlywheelI = 0.1; // Placeholder
  public static final double kFlywheelD = 0.0; // Placeholder

  // Field-specific constants
  public static final double FIELD_LENGTH = 16.54; // meters
  public static final double FIELD_WIDTH = 8.21;   // meters
  public static final double HUB_HEIGHT = 1.8288; // 72 inches in meters
  
  // Alliance-specific targets
  public static final Translation2d BLUE_HUB_POSITION = new Translation2d(4.5, FIELD_WIDTH / 2);
  public static final Translation2d RED_HUB_POSITION = new Translation2d(FIELD_LENGTH - 4.5, FIELD_WIDTH / 2);
  public static final Translation2d BLUE_DRIVER_STATION = new Translation2d(0.5, FIELD_WIDTH / 2);
  public static final Translation2d RED_DRIVER_STATION = new Translation2d(FIELD_LENGTH - 0.5, FIELD_WIDTH / 2);
}
