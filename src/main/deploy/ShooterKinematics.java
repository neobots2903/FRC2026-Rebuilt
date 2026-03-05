package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/**
 * ============================================================================ STATIONARY SHOOTER
 * KINEMATICS (COMPLETED)
 * ============================================================================ Calculates the exact
 * turret angle and hood angle needed to hit the Hub based purely on our robot's odometry (Pose2d)
 * on the field.
 */
public class ShooterKinematics {

  // =========================================================================
  // 1. RAW MEASUREMENTS (Human Readable)
  // =========================================================================

  // Toggle for our empirical Look-Up Table (LUT). Set to false until we tune on a real field!
  public static final boolean ENABLE_EMPIRICAL_LUT = false;

  // Field target (Meters) - UPDATE THESE to the real field hub coordinates!
  private static final double TARGET_X_METERS = 10.0;
  private static final double TARGET_Y_METERS = 10.0;
  private static final double TARGET_Z_METERS =
      3.0; // Changed from 10 to 3 meters (~10 ft) for a realistic FRC target

  // Robot measurements (Inches) - UPDATE THESE from CAD
  private static final double TURRET_OFFSET_X_IN = 10.0;
  private static final double TURRET_OFFSET_Y_IN = 0.0;
  private static final double SHOOTER_HEIGHT_IN = 16.0;
  private static final double WHEEL_RADIUS_IN = 2.0;

  // Target Geometry (Inches & Degrees) - To prevent bounce-outs!
  private static final double TARGET_DIAMETER_IN = 24.0;
  private static final double MIN_DESCENT_ANGLE_DEG =
      20.0; // Minimum downward angle to fit through the rim

  // Motor & Limits
  public static final double IDEAL_MOTOR_RPM = 3000.0;
  public static final double GEAR_REDUCTION = 1.0;
  private static final double MIN_HOOD_ANGLE_DEG = 0.0;
  private static final double MAX_HOOD_ANGLE_DEG = 75.0;
  private static final double MIN_TURRET_ANGLE_DEG = -150.0;
  private static final double MAX_TURRET_ANGLE_DEG = 150.0;

  // =========================================================================
  // 2. PHYSICS CONSTANTS (The Metric Zone)
  // =========================================================================

  private static final double GRAVITY = 9.81;

  private static final Translation3d TARGET_POSE_METERS =
      new Translation3d(TARGET_X_METERS, TARGET_Y_METERS, TARGET_Z_METERS);

  // Unit Conversions: Human inches -> Physics meters
  private static final Translation2d TURRET_OFFSET_METERS =
      new Translation2d(
          Units.inchesToMeters(TURRET_OFFSET_X_IN), Units.inchesToMeters(TURRET_OFFSET_Y_IN));
  private static final double SHOOTER_HEIGHT_METERS = Units.inchesToMeters(SHOOTER_HEIGHT_IN);
  private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(WHEEL_RADIUS_IN);

  // =========================================================================
  // 3. THE LOOK-UP TABLE (LUT)
  // =========================================================================

  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_MAP = new InterpolatingDoubleTreeMap();

  static {
    // DUMMY DATA: Replace when you test the real robot!
    // HOOD_ANGLE_MAP.put(distanceMeters, angleDegrees);
    // HOOD_ANGLE_MAP.put(2.0, 35.0);
  }

  // =========================================================================
  // 4. HELPER MATH METHODS
  // =========================================================================

  /** Calculates the exit velocity of the game piece in meters per second. */
  public static double getExitVelocityMetersPerSec() {
    double flywheelRpm = IDEAL_MOTOR_RPM / GEAR_REDUCTION;
    double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS_METERS;

    // Convert RPM to Surface Speed (m/s)
    double surfaceSpeedMetersPerSec = (flywheelRpm / 60.0) * wheelCircumference;

    // In a single-wheel hooded shooter, ball exits at ~half the surface speed
    return surfaceSpeedMetersPerSec / 2.0;
  }

  /** Finds the exact position of the turret on the field, accounting for robot rotation. */
  public static Translation2d getTurretFieldPosition(Pose2d robotPose) {
    // Rotate the offset by the robot's heading, then add it to the robot's coordinates
    Translation2d rotatedOffset = TURRET_OFFSET_METERS.rotateBy(robotPose.getRotation());
    return robotPose.getTranslation().plus(rotatedOffset);
  }

  /** Finds the 2D horizontal distance from the turret to the target. */
  public static double getDistanceToTargetMeters(Pose2d robotPose) {
    Translation2d turretPos = getTurretFieldPosition(robotPose);
    return turretPos.getDistance(
        new Translation2d(TARGET_POSE_METERS.getX(), TARGET_POSE_METERS.getY()));
  }

  // =========================================================================
  // 5. MAIN CONTROL METHODS
  // =========================================================================

  /** Calculates the required turret angle to face the target. */
  public static double calculateTurretAngleDeg(Pose2d robotPose) {
    Translation2d turretPos = getTurretFieldPosition(robotPose);

    double deltaX = TARGET_POSE_METERS.getX() - turretPos.getX();
    double deltaY = TARGET_POSE_METERS.getY() - turretPos.getY();

    // Find the absolute field angle to the target
    Rotation2d fieldAngleToTarget = new Rotation2d(Math.atan2(deltaY, deltaX));

    // Subtract robot rotation to make the angle relative to the chassis
    Rotation2d turretTargetRotation = fieldAngleToTarget.minus(robotPose.getRotation());
    double targetDegrees = turretTargetRotation.getDegrees();

    // Safety Check: Can our mechanism actually spin this far?
    if (targetDegrees < MIN_TURRET_ANGLE_DEG || targetDegrees > MAX_TURRET_ANGLE_DEG) {
      return Double.NaN; // Blind spot!
    }

    return targetDegrees;
  }

  /** Physics time. Uses the Projectile Motion Quadratic Equation to find the launch angle. */
  public static double getPhysicsHoodAngleDeg(Pose2d robotPose) {
    double distance = getDistanceToTargetMeters(robotPose);
    double deltaZ = TARGET_POSE_METERS.getZ() - SHOOTER_HEIGHT_METERS;
    double exitVelocity = getExitVelocityMetersPerSec();

    // The gravity constant component: k = (g * d^2) / (2 * v^2)
    double k = (GRAVITY * Math.pow(distance, 2)) / (2 * Math.pow(exitVelocity, 2));

    // Quadratic coefficients: A*tan^2(theta) + B*tan(theta) + C = 0
    double a = k;
    double b = -distance;
    double c = deltaZ + k;

    double discriminant = Math.pow(b, 2) - (4 * a * c);

    // If discriminant is negative, the shot is physically impossible at this velocity
    if (discriminant < 0) {
      return Double.NaN;
    }

    // Calculate both possible tangent values
    double tanTheta1 = (-b + Math.sqrt(discriminant)) / (2 * a);
    double tanTheta2 = (-b - Math.sqrt(discriminant)) / (2 * a);

    // Convert to degrees
    double angle1 = Math.toDegrees(Math.atan(tanTheta1));
    double angle2 = Math.toDegrees(Math.atan(tanTheta2));

    // Validate against hood limits AND the bounce-out check
    boolean angle1Valid =
        angle1 >= MIN_HOOD_ANGLE_DEG
            && angle1 <= MAX_HOOD_ANGLE_DEG
            && isTrajectoryValidForTopOpening(angle1, distance, exitVelocity);

    boolean angle2Valid =
        angle2 >= MIN_HOOD_ANGLE_DEG
            && angle2 <= MAX_HOOD_ANGLE_DEG
            && isTrajectoryValidForTopOpening(angle2, distance, exitVelocity);

    // FRC Strategy: Pick the flatter shot (smaller angle) IF it's valid
    if (angle1Valid && angle2Valid) {
      return Math.min(angle1, angle2);
    } else if (angle1Valid) {
      return angle1;
    } else if (angle2Valid) {
      return angle2;
    }

    return Double.NaN; // No valid angles fit the criteria
  }

  /**
   * Prevents the "Line-Drive Bounce-Out". Checks if a hood angle creates a shot that is falling
   * steeply enough into the top opening.
   */
  private static boolean isTrajectoryValidForTopOpening(
      double hoodAngleDeg, double distance, double exitVelocity) {
    double angleRad = Math.toRadians(hoodAngleDeg);

    // 1. Horizontal velocity (constant)
    double horizontalVelocity = exitVelocity * Math.cos(angleRad);

    // 2. Time of Flight (t = d / v_x)
    double timeOfFlight = distance / horizontalVelocity;

    // 3. Final vertical velocity (Vf = Vi - g*t)
    double initialVerticalVelocity = exitVelocity * Math.sin(angleRad);
    double finalVerticalVelocity = initialVerticalVelocity - (GRAVITY * timeOfFlight);

    // If final vertical velocity is positive, it's still going UP when it hits the goal.
    // Bounce-out!
    if (finalVerticalVelocity > 0) {
      return false;
    }

    // 4. Calculate Entry Angle. We use absolute value because vy is negative (falling)
    double entryAngleDeg =
        Math.toDegrees(Math.abs(Math.atan2(finalVerticalVelocity, horizontalVelocity)));

    // 5. Does the game piece enter steeply enough?
    return entryAngleDeg >= MIN_DESCENT_ANGLE_DEG;
  }

  /** Decides whether to use the empirical LUT or the physics math. */
  public static double getOptimalHoodAngleDeg(Pose2d robotPose) {
    // Fallback Logic: Only use the map if the toggle is ON and the map actually has data.
    if (ENABLE_EMPIRICAL_LUT && !HOOD_ANGLE_MAP.isEmpty()) {
      double distance = getDistanceToTargetMeters(robotPose);
      double targetAngle = HOOD_ANGLE_MAP.get(distance);

      // Safety check against mechanism limits
      if (targetAngle < MIN_HOOD_ANGLE_DEG || targetAngle > MAX_HOOD_ANGLE_DEG) {
        return Double.NaN;
      }
      return targetAngle;
    } else {
      return getPhysicsHoodAngleDeg(robotPose);
    }
  }
}
