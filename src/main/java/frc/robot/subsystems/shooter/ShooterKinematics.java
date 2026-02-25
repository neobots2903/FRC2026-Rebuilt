package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

/**
 * ============================================================================
 * ADVANCED SHOOTER KINEMATICS
 * ============================================================================
 * Goal: Calculate the exact turret angle and hood angle needed to hit the Hub
 * based purely on our robot's odometry (Pose2d) on the field, even while moving.
 * We think in INCHES and DEGREES normally. 
 * FRC libraries and these physics equations work in METERS and RADIANS. 
 * We must convert all human inputs into metric before doing math.
 */
public class ShooterKinematics {

    // =========================================================================
    // 1. RAW MEASUREMENTS (Human Readable)
    // =========================================================================

    /**
     * TODO #0: Take all the measurements below. These are all placeholders.
     * RESOURCE: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
     * * HINT 1: The hub coordinates and height can be found in the Field Coordinate System document on the FRC Game Manual.
     * * HINT 2: The robot measurements can be taken on the actual robot or from CAD.
     * * HINT 3: For the motor RPM, find the motor's datasheet, then apply the gear reduction.
     * * HINT 4: For the motor and gear reduction, bother Holden and Darius.
     */
    
    // Toggle for our empirical Look-Up Table (LUT)
    public static final boolean ENABLE_EMPIRICAL_LUT = false; 
    
    // Field target (Meters)
    private static final double TARGET_X_METERS = 10.0;
    private static final double TARGET_Y_METERS = 10.0;
    private static final double TARGET_Z_METERS = 10.0; 
    
    // Robot measurements (Inches)
    private static final double TURRET_OFFSET_X_IN = 10.0; 
    private static final double TURRET_OFFSET_Y_IN = 0.0;  
    private static final double SHOOTER_HEIGHT_IN = 16.0;
    private static final double WHEEL_RADIUS_IN = 2.0;

    // Target Geometry (Inches & Degrees) - To prevent bounce-outs!
    private static final double TARGET_DIAMETER_IN = 24.0;
    private static final double MIN_DESCENT_ANGLE_DEG = 20.0; 
    
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
    
    private static final Translation3d TARGET_POSE_METERS = new Translation3d(
        TARGET_X_METERS, TARGET_Y_METERS, TARGET_Z_METERS
    );
    
    /**
     * TODO #1: Convert the INCHES into METERS!
     * RESOURCE: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/util/Units.html
     * HINT: Look for the `Units.inchesToMeters(...)` method.
     */
    private static final Translation2d TURRET_OFFSET_METERS = new Translation2d(0.0, 0.0); // Fix me!
    private static final double SHOOTER_HEIGHT_METERS = 0.0; // Fix me!
    private static final double WHEEL_RADIUS_METERS = 0.0; // Fix me!

    // =========================================================================
    // 3. THE LOOK-UP TABLE (LUT)
    // =========================================================================
    
    /**
     * RESOURCE: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/interpolation/InterpolatingDoubleTreeMap.html
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
     * Calculates the exit velocity of the game piece.
     * HINT 1: Find the actual wheel RPM using the IDEAL_MOTOR_RPM and GEAR_REDUCTION.
     * HINT 2: Convert that RPM into surface speed (Meters per Second) using the wheel's circumference.
     * HINT 3: Because the ball rolls against a stationary hood, the exit velocity is HALF the surface speed.
     */
    public static double getExitVelocityMetersPerSec() {
        // TODO #2: Implement the math described in the hints above.
        return 0.0; 
    }

    /**
     * Finds where the turret actually is on the field.
     * RESOURCE: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/index.html
     * HINT: The turret is offset from the center. If the robot spins, the turret's field position changes.
     * Call TURRET_OFFSET_METERS.rotateBy(...) using the robot's current rotation, 
     * then add that result to the robot's current Translation2d.
     */
    public static Translation2d getTurretFieldPosition(Pose2d robotPose) {
        // TODO #3: Calculate the turret's true Translation2d on the field.
        return new Translation2d(); 
    }

    public static double getDistanceToTargetMeters(Pose2d robotPose, Translation2d targetPos) {
        Translation2d turretPos = getTurretFieldPosition(robotPose);
        return turretPos.getDistance(targetPos);
    }

    // =========================================================================
    // 5. MAIN CONTROL METHODS
    // =========================================================================

    /**
     * Calculates the required turret angle to face the target.
     * RESOURCE 1: https://www.mathsisfun.com/polar-cartesian-coordinates.html
     * RESOURCE 2: https://docs.oracle.com/en/java/javase/17/docs/api/java.base/java/lang/Math.html#atan2(double,double)
     * * HINT 1: Find the deltaX and deltaY between the turret and the targetPos.
     * HINT 2: Use Math.atan2(deltaY, deltaX) to find the field angle to the target. (Y goes first!)
     * HINT 3: Wrap that angle in a Rotation2d, then subtract the robot's current rotation.
     * HINT 4: Get the degrees. If it's outside MIN_TURRET_ANGLE_DEG and MAX_TURRET_ANGLE_DEG, return Double.NaN!
     */
    public static double calculateTurretAngleDeg(Pose2d robotPose, Translation2d targetPos) {
        // TODO #4: Implement Turret Aiming Logic
        return Double.NaN; 
    }

    /**
     * Uses the Projectile Motion Quadratic Equation to find the launch angle.
     * RESOURCE 1: https://www.khanacademy.org/science/physics/two-dimensional-motion/two-dimensional-projectile-mot/a/what-is-2d-projectile-motion
     * RESOURCE 2: https://www.mathsisfun.com/algebra/quadratic-equation.html
     * * The equation is: A*tan^2(theta) + B*tan(theta) + C = 0
     * Let k = (GRAVITY * distance^2) / (2 * exitVelocity^2)
     * A = k
     * B = -distance
     * C = (targetHeight - shooterHeight) + k
     * * HINT 1: Calculate the discriminant (B^2 - 4AC). If negative, return Double.NaN (shot impossible).
     * HINT 2: Use the quadratic formula to find tan(theta), then use Math.atan() to get the actual angles in radians.
     * HINT 3: Convert the angles to degrees (Math.toDegrees), check if they fit inside our hood limits.
     * HINT 4: Check if the angle passes isTrajectoryValidForTopOpening() before accepting it!
     * HINT 5: If both fit and are valid, return the SMALLER angle (flatter shot). 
     */
    public static double getPhysicsHoodAngleDeg(Pose2d robotPose, Translation2d targetPos) {
        // TODO #5: Implement the quadratic formula projectile motion solver
        return Double.NaN; 
    }

    /**
     * Prevents the "Line-Drive Bounce-Out". 
     * 
     * Checks if a hood angle creates a shot that is falling steeply enough into the top opening.
     * * HINT 1: Find Time of Flight (t = distance / horizontal_velocity). 
     * Horizontal velocity is exitVelocity * Math.cos(angleInRadians).
     * HINT 2: Find final vertical velocity (Vf = Vi - gravity * t).
     * Initial vertical velocity (Vi) is exitVelocity * Math.sin(angleInRadians).
     * HINT 3: If final vertical velocity is > 0, the ball is going UP. Return false!
     * HINT 4: Find entry angle using Math.toDegrees(Math.abs(Math.atan2(finalVerticalVelocity, horizontalVelocity))).
     * HINT 5: Return true ONLY if entry angle >= MIN_DESCENT_ANGLE_DEG.
     */
    private static boolean isTrajectoryValidForTopOpening(double hoodAngleDeg, double distance, double exitVelocity) {
        // TODO #6: Implement the Falling/Steepness Check
        return false;
    }

    /**
     * Decides whether to use the empirical LUT or the physics math.
     * HINT: If ENABLE_EMPIRICAL_LUT is true AND the HOOD_ANGLE_MAP is not empty (!HOOD_ANGLE_MAP.isEmpty()), 
     * get the distance to the target and return HOOD_ANGLE_MAP.get(distance).
     * Otherwise, call getPhysicsHoodAngleDeg() and return its result.
     */
    public static double getOptimalHoodAngleDeg(Pose2d robotPose, Translation2d targetPos) {
        // TODO #7: Implement Graceful Degradation logic
        return Double.NaN; 
    }
}