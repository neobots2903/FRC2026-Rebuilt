package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/**
 * ============================================================================ ADVANCED SHOOTER
 * KINEMATICS ============================================================================ Goal:
 * Calculate the exact turret angle and hood angle needed to hit the Hub based purely on our robot's
 * odometry (Pose2d) on the field, even while moving. We think in INCHES and DEGREES normally. FRC
 * libraries and these physics equations work in METERS and RADIANS. We must convert all human
 * inputs into metric before doing math.
 */
public class ShooterKinematicsOld {

  // =========================================================================
  // 1. RAW MEASUREMENTS (Human Readable)
  // =========================================================================

  /**
   * RESOURCE:
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html * HINT
   * 1: The hub coordinates and height can be found in the Field Coordinate System document on the
   * FRC Game Manual. * HINT 2: The robot measurements can be taken on the actual robot or from CAD.
   * * HINT 3: For the motor RPM, find the motor's datasheet, then apply the gear reduction. * HINT
   * 4: For the motor and gear reduction, bother Holden and Darius.
   */

  // Toggle for our empirical Look-Up Table (LUT)
  public static final boolean ENABLE_EMPIRICAL_LUT = false;

  // Robot measurements (Inches)
  private static final double TURRET_OFFSET_X_IN = 7.0;
  private static final double TURRET_OFFSET_Y_IN = 0.0;
  private static final double SHOOTER_HEIGHT_IN = 15.15;
  private static final double WHEEL_RADIUS_IN = 2.0; // Theoretically

  // Target Geometry (Inches & Degrees) - To prevent bounce-outs!
  private static final double TARGET_DIAMETER_IN = 11.76;
  private static final double MIN_DESCENT_ANGLE_DEG = 20.0; // !Test

  // Motor & Limits
  public static final double IDEAL_MOTOR_RPM = 5676.0; // !Darius's guess
  public static final double GEAR_REDUCTION =
      3.0; // !Holden doesn't know, Darius guesses it will proably be 3/1
  private static final double MIN_HOOD_ANGLE_DEG = 30.0;
  private static final double MAX_HOOD_ANGLE_DEG = 63.0; // Theoretically 70
  private static final double MIN_TURRET_ANGLE_DEG = -270.0; // Theoretically -360
  private static final double MAX_TURRET_ANGLE_DEG = 270.0; // Theoretically 360

  // =========================================================================
  // 2. PHYSICS CONSTANTS (The Metric Zone)
  // =========================================================================

  private static final double GRAVITY = 9.81;

  private static final Translation3d TARGET_POSE_METERS = shooterConstants.TARGET_POSE_METERS;

  /**
   * RESOURCE:
   * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/util/Units.html HINT:
   * Look for the `Units.inchesToMeters(...)` method.
   */
  private static final Translation2d TURRET_OFFSET_METERS =
      new Translation2d(Units.inchesToMeters(10.0), Units.inchesToMeters(0.0));

  private static final double SHOOTER_HEIGHT_METERS = Units.inchesToMeters(16.0);
  private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(4.0);

  // =========================================================================
  // 3. THE LOOK-UP TABLE (LUT)
  // =========================================================================

  /**
   * RESOURCE:
   * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/interpolation/InterpolatingDoubleTreeMap.html
   */
  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_MAP = new InterpolatingDoubleTreeMap();

  static {
    // We will put real field data here later when the shooter is done.
    // HOOD_ANGLE_MAP.put(distanceMeters, angleDegrees);
  }

  // =========================================================================
  // 4. HELPER MATH METHODS
  // =========================================================================

  /**
   * Calculates the exit velocity of the game piece. HINT 1: Find the actual wheel RPM using the
   * IDEAL_MOTOR_RPM and GEAR_REDUCTION. HINT 2: Convert that RPM into surface speed (Meters per
   * Second) using the wheel's circumference. HINT 3: Because the ball rolls against a stationary
   * hood, the exit velocity is HALF the surface speed.
   */
  public static double getExitVelocityMetersPerSec() {
    // TODO #2: Implement the math described in the hints above.
    double realRPM = IDEAL_MOTOR_RPM / GEAR_REDUCTION;
    double wheelCircumference = 2 * WHEEL_RADIUS_METERS * Math.PI;
    double surfaceSpeed = (wheelCircumference * realRPM) / 2;
    return 0.0;
  }

  /**
   * Finds where the turret actually is on the field. RESOURCE:
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/index.html HINT: The
   * turret is offset from the center. If the robot spins, the turret's field position changes. Call
   * TURRET_OFFSET_METERS.rotateBy(...) using the robot's current rotation, then add that result to
   * the robot's current Translation2d.
   */
  public static Translation2d getTurretFieldPosition(Pose2d robotPose) {
    // TODO #3: Calculate the turret's true Translation2d on the field.
    Translation2d turretRotation = TURRET_OFFSET_METERS.rotateBy(robotPose.getRotation());
    Translation2d turretPosition = robotPose.getTranslation().plus(turretRotation);
    return turretPosition;
  }

  public static double getDistanceToTargetMeters(Pose2d robotPose, Translation2d targetPos) {
    Translation2d turretPos = getTurretFieldPosition(robotPose);
    return turretPos.getDistance(targetPos);
  }

  // =========================================================================
  // 5. MAIN CONTROL METHODS
  // =========================================================================

  /**
   * Calculates the required turret angle to face the target. RESOURCE 1:
   * https://www.mathsisfun.com/polar-cartesian-coordinates.html RESOURCE 2:
   * https://docs.oracle.com/en/java/javase/17/docs/api/java.base/java/lang/Math.html#atan2(double,double)
   * * HINT 1: Find the deltaX and deltaY between the turret and the targetPos. HINT 2: Use
   * Math.atan2(deltaY, deltaX) to find the field angle to the target. (Y goes first!) HINT 3: Wrap
   * that angle in a Rotation2d, then subtract the robot's current rotation. HINT 4: Get the
   * degrees. If it's outside MIN_TURRET_ANGLE_DEG and MAX_TURRET_ANGLE_DEG, return Double.NaN!
   */
  public static double calculateTurretAngleDeg(Pose2d robotPose) {
    // TODO #4: Implement Turret Aiming Logic
    Translation2d turretPosition = getTurretFieldPosition(robotPose);
    double deltaX = shooterConstants.TARGET_X_METERS - turretPosition.getX();
    double deltaY = shooterConstants.TARGET_Y_METERS - turretPosition.getY();
    Rotation2d fieldTargetAngle = new Rotation2d(Math.atan2(deltaY, deltaX));
    if (fieldTargetAngle.getDegrees() < MIN_TURRET_ANGLE_DEG
        || fieldTargetAngle.getDegrees() > MAX_TURRET_ANGLE_DEG) {
      return Double.NaN;
    }
    return fieldTargetAngle.getDegrees();
  }

  /**
   * Uses the Projectile Motion Quadratic Equation to find the launch angle. RESOURCE 1:
   * https://www.khanacademy.org/science/physics/two-dimensional-motion/two-dimensional-projectile-mot/a/what-is-2d-projectile-motion
   * RESOURCE 2: https://www.mathsisfun.com/algebra/quadratic-equation.html * The equation is:
   * A*tan^2(theta) + B*tan(theta) + C = 0 Let k = (GRAVITY * distance^2) / (2 * exitVelocity^2) A =
   * k B = -distance C = (targetHeight - shooterHeight) + k * HINT 1: Calculate the discriminant
   * (B^2 - 4AC). If negative, return Double.NaN (shot impossible). HINT 2: Use the quadratic
   * formula to find tan(theta), then use Math.atan() to get the actual angles in radians. HINT 3:
   * Convert the angles to degrees (Math.toDegrees), check if they fit inside our hood limits. HINT
   * 4: Check if the angle passes isTrajectoryValidForTopOpening() before accepting it! HINT 5: If
   * both fit and are valid, return the SMALLER angle (flatter shot).
   */
  public static double getPhysicsHoodAngleDeg(Pose2d robotPose, Translation2d targetPos) {
    double exitVelocity = getExitVelocityMetersPerSec();
    double distance = getDistanceToTargetMeters(robotPose, targetPos);
    double deltaZ = shooterConstants.TARGET_Z_METERS - SHOOTER_HEIGHT_METERS;
    double k = ((9.8 * Math.pow(distance, 2)) / (2 * Math.pow(exitVelocity, 2)));
    double B = -distance;
    double C = deltaZ + k;
    double discriminant = (Math.pow(B, 2) - 4 * k * C);
    if (discriminant < 0) {
      return Double.NaN;
    }
    double tanTheta1 = (-B + Math.sqrt(discriminant)) / (2 * k);
    double tanTheta2 = (-B - Math.sqrt(discriminant)) / (2 * k);
    double angle1 = Math.toDegrees(Math.atan(tanTheta1));
    double angle2 = Math.toDegrees(Math.atan(tanTheta2));
    boolean angle1Valid =
        angle1 >= MIN_HOOD_ANGLE_DEG
            && angle1 <= MAX_HOOD_ANGLE_DEG
            && isTrajectoryValidForTopOpening(angle1, distance, exitVelocity);
    boolean angle2Valid =
        angle2 >= MIN_HOOD_ANGLE_DEG
            && angle2 <= MAX_HOOD_ANGLE_DEG
            && isTrajectoryValidForTopOpening(angle2, distance, exitVelocity);
    if (angle1Valid && angle2Valid) {
      return Math.min(angle1, angle2);
    } else if (angle1Valid) {
      return angle1;
    } else if (angle2Valid) {
      return angle2;
    } else {
      return Double.NaN;
    }
  }

  /**
   * Prevents the "Line-Drive Bounce-Out".
   *
   * <p>Checks if a hood angle creates a shot that is falling steeply enough into the top opening. *
   * HINT 1: Find Time of Flight (t = distance / horizontal_velocity). Horizontal velocity is
   * exitVelocity * Math.cos(angleInRadians). HINT 2: Find final vertical velocity (Vf = Vi -
   * gravity * t). Initial vertical velocity (Vi) is exitVelocity * Math.sin(angleInRadians). HINT
   * 3: If final vertical velocity is > 0, the ball is going UP. Return false! HINT 4: Find entry
   * angle using Math.toDegrees(Math.abs(Math.atan2(finalVerticalVelocity, horizontalVelocity))).
   * HINT 5: Return true ONLY if entry angle >= MIN_DESCENT_ANGLE_DEG.
   */
  private static boolean isTrajectoryValidForTopOpening(
      double hoodAngleDeg, double distance, double exitVelocity) {
    double horizVelocity = exitVelocity * Math.cos(Math.toRadians(hoodAngleDeg));
    double initVertiVelocity = exitVelocity * Math.sin(Math.toRadians(hoodAngleDeg));
    double timeOfFlight = distance / horizVelocity;
    double finalVertiVelocity = initVertiVelocity - 9.8 * timeOfFlight;
    if (finalVertiVelocity > 0) {
      return false;
    }
    double entryAngle = Math.toDegrees(Math.abs(Math.atan2(finalVertiVelocity, horizVelocity)));
    if (entryAngle >= MIN_DESCENT_ANGLE_DEG) {
      return true;
    }
    return false;
  }

  /**
   * Decides whether to use the empirical LUT or the physics math. HINT: If ENABLE_EMPIRICAL_LUT is
   * true AND the HOOD_ANGLE_MAP is not empty (!HOOD_ANGLE_MAP.isEmpty()), get the distance to the
   * target and return HOOD_ANGLE_MAP.get(distance). Otherwise, call getPhysicsHoodAngleDeg() and
   * return its result.
   */
  public static double getOptimalHoodAngleDeg(Pose2d robotPose, Translation2d targetPos) {
    if (ENABLE_EMPIRICAL_LUT) { //  && !HOOD_ANGLE_MAP.isEmpty()
      double distance = getDistanceToTargetMeters(robotPose, targetPos);
      return HOOD_ANGLE_MAP.get(distance);
    } else {
      double angle = getPhysicsHoodAngleDeg(robotPose, targetPos);
      return angle;
    }
  }
}
