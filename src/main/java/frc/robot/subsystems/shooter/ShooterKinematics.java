package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * ============================================================================ ADVANCED SHOOTER
 * KINEMATICS ============================================================================ Goal:
 * Calculate the exact turret angle AND flywheel RPM needed to hit the Hub based on our robot's
 * odometry (Pose2d) on the field. Unlike the old version which fixed RPM and only solved for angle,
 * this version solves for BOTH — sweeping exit velocities from the minimum required up to the
 * motor's maximum, picking the first valid (lowest-energy) solution.
 */
public class ShooterKinematics {

  // =========================================================================
  // 1. RAW MEASUREMENTS (Human Readable)
  // =========================================================================

  // Toggle for our empirical Look-Up Table (LUT)
  public static final boolean ENABLE_EMPIRICAL_LUT = false;

  // Robot measurements (Inches)
  private static final double TURRET_OFFSET_X_IN = 7.0;
  private static final double TURRET_OFFSET_Y_IN = 0.0;
  private static final double SHOOTER_HEIGHT_IN = 15.15;
  private static final double WHEEL_RADIUS_IN = 2.0;

  // Target Geometry (Inches & Degrees) - To prevent bounce-outs!
  private static final double TARGET_DIAMETER_IN = 11.76;
  private static final double MIN_DESCENT_ANGLE_DEG = 20.0;

  // Motor & Limits
  public static final double IDEAL_MOTOR_RPM = 3000.0;
  public static final double MAX_MOTOR_RPM = 6000.0;
  public static final double GEAR_REDUCTION = 1.0;
  private static final double MIN_HOOD_ANGLE_DEG = shooterConstants.kMinHoodAngle;
  private static final double MAX_HOOD_ANGLE_DEG = shooterConstants.kMaxHoodAngle;
  private static final double MIN_TURRET_ANGLE_DEG = -270.0;
  private static final double MAX_TURRET_ANGLE_DEG = 270.0;

  // Velocity sweep parameters
  private static final double MIN_EXIT_VELOCITY_MPS = 5.0;
  private static final double VELOCITY_STEP_MPS = 0.5;
  private static final double IDEAL_LAUNCH_ANGLE_DEG = 45.0;

  // =========================================================================
  // 2. PHYSICS CONSTANTS (The Metric Zone)
  // =========================================================================

  private static final double GRAVITY = 9.81;

  private static final Translation2d TURRET_OFFSET_METERS =
      new Translation2d(
          Units.inchesToMeters(TURRET_OFFSET_X_IN), Units.inchesToMeters(TURRET_OFFSET_Y_IN));

  private static final double SHOOTER_HEIGHT_METERS = Units.inchesToMeters(SHOOTER_HEIGHT_IN);
  private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(WHEEL_RADIUS_IN);

  public static Translation3d getTargetTowerPosition() {
    var alliance = DriverStation.getAlliance();
    boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
    return isBlue ? shooterConstants.BLUE_HUB_POSITION : shooterConstants.RED_HUB_POSITION;
  }

  // =========================================================================
  // 3. THE LOOK-UP TABLE (LUT)
  // =========================================================================

  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_MAP = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_MAP =
      new InterpolatingDoubleTreeMap();

  static {
    // We will put real field data here later when the shooter is done.
    // Both maps are keyed by distance (meters).
    // HOOD_ANGLE_MAP.put(distanceMeters, angleDegrees);
    // FLYWHEEL_RPM_MAP.put(distanceMeters, motorRPM);
    //
    // Example (uncomment and fill with real data):
    // HOOD_ANGLE_MAP.put(2.0, 55.0);    FLYWHEEL_RPM_MAP.put(2.0, 2500.0);
    // HOOD_ANGLE_MAP.put(3.0, 45.0);    FLYWHEEL_RPM_MAP.put(3.0, 3000.0);
    // HOOD_ANGLE_MAP.put(5.0, 38.0);    FLYWHEEL_RPM_MAP.put(5.0, 3800.0);
    // HOOD_ANGLE_MAP.put(7.0, 33.0);    FLYWHEEL_RPM_MAP.put(7.0, 4500.0);
    // HOOD_ANGLE_MAP.put(10.0, 28.0);   FLYWHEEL_RPM_MAP.put(10.0, 5500.0);
  }

  // =========================================================================
  // 4. SHOOTER SOLUTION CLASS
  // =========================================================================

  /** Holds a complete shooter solution: both the hood angle and flywheel RPM. */
  public static class ShooterSolution {
    public final double hoodAngleDeg;
    public final double motorRPM;
    public final boolean isValid;

    public ShooterSolution(double hoodAngleDeg, double motorRPM, boolean isValid) {
      this.hoodAngleDeg = hoodAngleDeg;
      this.motorRPM = motorRPM;
      this.isValid = isValid;
    }

    public static final ShooterSolution INVALID = new ShooterSolution(0, 0, false);
  }

  // =========================================================================
  // 5. HELPER MATH METHODS
  // =========================================================================

  /**
   * Calculates the MAXIMUM exit velocity of the game piece at full motor power.
   *
   * <p>HINT 1: Find the actual wheel RPM: MAX_MOTOR_RPM / GEAR_REDUCTION. HINT 2: Convert RPM to
   * revolutions per SECOND by dividing by 60. HINT 3: Multiply by wheel circumference (2 * π *
   * radius) to get surface speed in m/s. HINT 4: Divide by 2 because the ball rolls against a
   * stationary hood.
   */
  public static double getMaxExitVelocityMetersPerSec() {
    double realRPM = MAX_MOTOR_RPM / GEAR_REDUCTION;
    double wheelCircumference = 2 * WHEEL_RADIUS_METERS * Math.PI;
    double surfaceSpeedMetersPerSec = (realRPM / 60.0) * wheelCircumference;
    return surfaceSpeedMetersPerSec / 2.0;
  }

  /**
   * Converts an exit velocity (m/s) BACK to the motor RPM needed to achieve it. This is the inverse
   * of getMaxExitVelocityMetersPerSec().
   *
   * <p>HINT 1: The exit velocity is half the surface speed, so: surfaceSpeed = exitVelocity * 2
   * HINT 2: surfaceSpeed = (wheelRPM / 60) * wheelCircumference, so solve for wheelRPM. HINT 3:
   * motorRPM = wheelRPM * GEAR_REDUCTION.
   */
  public static double exitVelocityToMotorRPM(double exitVelocityMps) {
    double surfaceSpeed = exitVelocityMps * 2.0;
    double wheelRPM = 60 * (surfaceSpeed / (2 * Math.PI * WHEEL_RADIUS_METERS));
    double motorRPM = wheelRPM * GEAR_REDUCTION;
    return motorRPM;
  }

  /**
   * Computes the MINIMUM exit velocity needed to reach a target at (distance, deltaZ).
   *
   * <p>From the quadratic discriminant ≥ 0 condition, we derive: v_min = sqrt(g * (deltaZ +
   * sqrt(deltaZ² + distance²)))
   *
   * <p>HINT: This is a direct formula — just plug in the values!
   */
  public static double getMinExitVelocity(double distance, double deltaZ) {
    double minimumExitVelocity =
        Math.sqrt(9.8 * (deltaZ + Math.sqrt(Math.pow(deltaZ, 2) + Math.pow(distance, 2))));
    return minimumExitVelocity;
  }

  /** Finds where the turret actually is on the field. */
  public static Translation2d getTurretFieldPosition(Pose2d robotPose) {
    Translation2d turretRotation = TURRET_OFFSET_METERS.rotateBy(robotPose.getRotation());
    Translation2d turretPosition = robotPose.getTranslation().plus(turretRotation);
    return turretPosition;
  }

  /** Calculates distance from the turret to the target. */
  public static double getDistanceToTargetMeters(Pose2d robotPose, Translation2d targetPos) {
    Translation2d turretPos = getTurretFieldPosition(robotPose);
    return turretPos.getDistance(targetPos);
  }

  // =========================================================================
  // 6. TURRET AIMING
  // =========================================================================

  public static double calculateTurretAngleDeg(Pose2d robotPose) {
    Translation2d turretPosition = getTurretFieldPosition(robotPose);
    Translation3d target = getTargetTowerPosition();
    double deltaX = target.getX() - turretPosition.getX();
    double deltaY = target.getY() - turretPosition.getY();
    Rotation2d fieldTargetAngle = new Rotation2d(Math.atan2(deltaY, deltaX));
    if (fieldTargetAngle.getDegrees() < MIN_TURRET_ANGLE_DEG
        || fieldTargetAngle.getDegrees() > MAX_TURRET_ANGLE_DEG) {
      return Double.NaN;
    }
    return fieldTargetAngle.getDegrees();
  }

  // =========================================================================
  // 7. HOOD ANGLE SOLVER (for a given velocity)
  // =========================================================================

  /**
   * Solves the projectile quadratic for hood angle at a SPECIFIC exit velocity.
   *
   * <p>This is the same math as before, but now it takes exitVelocity as a PARAMETER instead of
   * always using the max.
   *
   * <p>HINT 1: Calculate k = (GRAVITY * distance²) / (2 * exitVelocity²) HINT 2: Set up the
   * quadratic: A = k, B = -distance, C = deltaZ + k HINT 3: Compute discriminant = B² - 4*A*C. If <
   * 0, return Double.NaN. HINT 4: Find both angles using atan(quadratic formula). HINT 5: Check
   * each angle against hood limits AND isTrajectoryValidForTopOpening(). HINT 6: If both valid,
   * return the one CLOSEST to IDEAL_LAUNCH_ANGLE_DEG (45°). HINT 7: If only one valid, return that
   * one. If neither, return Double.NaN.
   */
  private static double solveHoodAngleForVelocity(
      double exitVelocity, double distance, double deltaZ) {

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
      angle1 = Math.abs(angle1 - IDEAL_LAUNCH_ANGLE_DEG);
      angle2 = Math.abs(angle2 - IDEAL_LAUNCH_ANGLE_DEG);
      return Math.min(angle1, angle2);
    } else if (angle1Valid) {
      return angle1;
    } else if (angle2Valid) {
      return angle2;
    } else {
      return Double.NaN;
    }
  }

  // =========================================================================
  // 8. THE MAIN SOLVER — Sweeps velocity to find a solution
  // =========================================================================

  /**
   * The main entry point. Sweeps exit velocities from the minimum required up to the motor's
   * maximum, and returns the first (lowest-energy) valid ShooterSolution.
   *
   * <p>HINT 1: Use getDistanceToTargetMeters() and compute deltaZ. HINT 2: Use
   * getMaxExitVelocityMetersPerSec() for the ceiling. HINT 3: Use getMinExitVelocity() for the
   * floor. Clamp it to at least MIN_EXIT_VELOCITY_MPS. HINT 4: If minVelocity > maxVelocity, return
   * ShooterSolution.INVALID (target unreachable). HINT 5: Loop from minVelocity to maxVelocity in
   * steps of VELOCITY_STEP_MPS. HINT 6: At each step, call solveHoodAngleForVelocity(). If it
   * returns a valid angle (not NaN): - Convert velocity to RPM with exitVelocityToMotorRPM() -
   * Return new ShooterSolution(angle, rpm, true) HINT 7: After the loop, also try maxVelocity as a
   * last resort. HINT 8: If nothing works, return ShooterSolution.INVALID.
   */
  public static ShooterSolution getShooterSolution(Pose2d robotPose, Translation2d targetPos) {
    double distance = getDistanceToTargetMeters(robotPose, targetPos);
    double deltaZ = getTargetTowerPosition().getZ() - SHOOTER_HEIGHT_METERS;
    double maxVelocity = getMaxExitVelocityMetersPerSec();
    double minVelocity = getMinExitVelocity(distance, deltaZ);

    minVelocity = Math.max(minVelocity, MIN_EXIT_VELOCITY_MPS);
    if (minVelocity > maxVelocity) {
      return ShooterSolution.INVALID;
    }

    for (double v = minVelocity; v <= maxVelocity; v += VELOCITY_STEP_MPS) {
      double angle = solveHoodAngleForVelocity(v, distance, deltaZ);
      if (!Double.isNaN(angle)) {
        double rpm = exitVelocityToMotorRPM(v);
        return new ShooterSolution(angle, rpm, true);
      }
    }
    // Try max velocity as last resort
    double angle = solveHoodAngleForVelocity(maxVelocity, distance, deltaZ);
    if (!Double.isNaN(angle)) {
      double rpm = exitVelocityToMotorRPM(maxVelocity);
      return new ShooterSolution(angle, rpm, true);
    }
    return ShooterSolution.INVALID;
  }

  // =========================================================================
  // 9. PUBLIC API — What the rest of the robot calls
  // =========================================================================

  /**
   * Returns a full ShooterSolution (angle + RPM). Uses the LUT if enabled, otherwise uses the
   * physics solver.
   *
   * <p>HINT: If LUT is enabled and not empty, use it. Otherwise, call getShooterSolution().
   */
  public static ShooterSolution getOptimalSolution(Pose2d robotPose, Translation2d targetPos) {
    if (ENABLE_EMPIRICAL_LUT) {
      double distance = getDistanceToTargetMeters(robotPose, targetPos);
      double angle = HOOD_ANGLE_MAP.get(distance);
      double rpm = FLYWHEEL_RPM_MAP.get(distance);
      return new ShooterSolution(angle, rpm, true);
    } else {
      return getShooterSolution(robotPose, targetPos);
    }
  }

  // =========================================================================
  // 10. TRAJECTORY VALIDATION
  // =========================================================================

  /**
   * Prevents the "Line-Drive Bounce-Out". Checks if a hood angle creates a shot that is falling
   * steeply enough into the top opening.
   */
  private static boolean isTrajectoryValidForTopOpening(
      double hoodAngleDeg, double distance, double exitVelocity) {
    double horizVelocity = exitVelocity * Math.cos(Math.toRadians(hoodAngleDeg));
    double initVertiVelocity = exitVelocity * Math.sin(Math.toRadians(hoodAngleDeg));
    double timeOfFlight = distance / horizVelocity;
    double finalVertiVelocity = initVertiVelocity - GRAVITY * timeOfFlight;
    if (finalVertiVelocity > 0) {
      return false;
    }
    double entryAngle = Math.toDegrees(Math.abs(Math.atan2(finalVertiVelocity, horizVelocity)));
    return entryAngle >= MIN_DESCENT_ANGLE_DEG;
  }
}
