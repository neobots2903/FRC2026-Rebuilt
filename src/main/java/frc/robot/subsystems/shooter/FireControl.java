package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.firecontrol.ProjectileSimulator;
import frc.firecontrol.ShotCalculator;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Wraps the frc-fire-control library (ProjectileSimulator + ShotCalculator) with our robot-specific
 * measurements from CAD. Sweeps hood angles from {@link shooterConstants#kMinHoodAngle} to {@link
 * shooterConstants#kMaxHoodAngle} and picks the angle with the lowest RPM among valid solutions.
 *
 * <p>Call {@link #initialize()} once at startup, then call {@link #calculate} every cycle.
 */
public class FireControl {

  // ── Robot-specific measurements (from CAD) ──────────────────────────────

  // Ball properties (2026 REBUILT game piece)
  private static final double BALL_MASS_KG = 0.215;
  private static final double BALL_DIAMETER_M = 0.1501;
  private static final double DRAG_COEFF = 0.47; // smooth sphere
  private static final double MAGNUS_COEFF = 0.2;
  private static final double AIR_DENSITY = 1.225;

  // Shooter geometry — measure these from YOUR CAD
  private static final double EXIT_HEIGHT_M =
      Units.inchesToMeters(15); // floor to ball center at exit
  private static final double WHEEL_DIAMETER_M =
      Units.inchesToMeters(6); // flywheel wheel diameter (6")
  private static final double SLIP_FACTOR = 0.3; // 0=no grip, 1=perfect;

  // Launcher position relative to robot center (meters)
  private static final double LAUNCHER_OFFSET_X = -Units.inchesToMeters(7); // ~7 inches backwards
  private static final double LAUNCHER_OFFSET_Y = 0.0; // centered

  // Sim parameters
  private static final double SIM_DT = 0.001;
  private static final double RPM_MIN = 1500;
  private static final double RPM_MAX = 6000;
  private static final int BINARY_SEARCH_ITERS = 25;
  private static final double MAX_SIM_TIME = 5.0;

  // Hood sweep parameters — 3° steps from kMinHoodAngle to kMaxHoodAngle
  // Gives 5 slots: 21, 24, 27, 30, 33 (plus 35 if it lands on max)
  private static final double HOOD_ANGLE_STEP_DEG = 3.0;

  // Minimum confidence for a hood-angle solution to be considered during sweep.
  // Among all solutions above this threshold, we pick the lowest RPM.
  private static final double MIN_SWEEP_CONFIDENCE = 20.0;

  // ── Field targets ───────────────────────────────────────────────────────

  private static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.5, 4.105);
  private static final Translation2d RED_HUB_CENTER =
      new Translation2d(shooterConstants.FIELD_LENGTH - 4.5, 4.105);

  // hubForward must point FROM the hub TOWARD the alliance's side of the field.
  // The ShotCalculator rejects shots where dot(hub-robot, hubForward) < 0,
  // i.e. the robot is "behind" the hub.
  // Blue robots approach from X < hubX, so (hub - robot) points +X → hubForward = (+1, 0)
  // Red robots approach from X > hubX, so (hub - robot) points -X → hubForward = (-1, 0)
  private static final Translation2d BLUE_HUB_FORWARD = new Translation2d(1, 0);
  private static final Translation2d RED_HUB_FORWARD = new Translation2d(-1, 0);

  // ── Per-angle shot calculators ──────────────────────────────────────────

  /** One ShotCalculator per hood angle, each loaded with its own LUT. */
  private static class AngleSlot {
    final double hoodAngleDeg;
    final ShotCalculator shotCalc;

    AngleSlot(double hoodAngleDeg, ShotCalculator shotCalc) {
      this.hoodAngleDeg = hoodAngleDeg;
      this.shotCalc = shotCalc;
    }
  }

  /** Firing solution that includes which hood angle won. */
  public static class FireControlResult {
    public final ShotCalculator.LaunchParameters launchParams;
    public final double hoodAngleDeg;

    public FireControlResult(ShotCalculator.LaunchParameters launchParams, double hoodAngleDeg) {
      this.launchParams = launchParams;
      this.hoodAngleDeg = hoodAngleDeg;
    }

    public static final FireControlResult INVALID =
        new FireControlResult(
            ShotCalculator.LaunchParameters.INVALID, shooterConstants.kMinHoodAngle);
  }

  // ── State ───────────────────────────────────────────────────────────────

  private final List<AngleSlot> angleSlots = new ArrayList<>();
  private boolean initialized = false;

  public FireControl() {}

  /**
   * Generate a LUT and ShotCalculator for every hood angle in the sweep range. Call once during
   * robotInit/constructors. With 3 angles × 91 distances this takes ~1-2s but only runs once.
   */
  public void initialize() {
    long totalStart = System.currentTimeMillis();
    int totalLoaded = 0;

    double minAngle = shooterConstants.kMinHoodAngle;
    double maxAngle = shooterConstants.kMaxHoodAngle;

    for (double hoodAngle = minAngle; hoodAngle <= maxAngle; hoodAngle += HOOD_ANGLE_STEP_DEG) {
      // Convert hood angle to launch angle from horizontal.
      // Hood 20° (all the way down) = 70° launch, hood 35° = 55° launch.
      double launchAngle = 90.0 - hoodAngle;

      // Build sim params for this launch angle
      ProjectileSimulator.SimParameters simParams =
          new ProjectileSimulator.SimParameters(
              BALL_MASS_KG,
              BALL_DIAMETER_M,
              DRAG_COEFF,
              MAGNUS_COEFF,
              AIR_DENSITY,
              EXIT_HEIGHT_M,
              WHEEL_DIAMETER_M,
              shooterConstants.TARGET_Z_METERS,
              SLIP_FACTOR,
              launchAngle, // ← actual launch angle from horizontal
              SIM_DT,
              RPM_MIN,
              RPM_MAX,
              BINARY_SEARCH_ITERS,
              MAX_SIM_TIME);

      ProjectileSimulator sim = new ProjectileSimulator(simParams);
      var lut = sim.generateLUT();

      // Configure a ShotCalculator for this angle
      ShotCalculator.Config config = new ShotCalculator.Config();
      config.launcherOffsetX = LAUNCHER_OFFSET_X;
      config.launcherOffsetY = LAUNCHER_OFFSET_Y;
      config.phaseDelayMs = 30.0;
      config.mechLatencyMs = 20.0;
      config.maxTiltDeg = 5.0;
      config.headingSpeedScalar = 1.0;
      config.headingReferenceDistance = 2.5;
      config.minScoringDistance = 0.8;
      config.maxScoringDistance = 5.0;

      ShotCalculator calc = new ShotCalculator(config);
      int loaded = 0;
      for (var entry : lut.entries()) {
        if (entry.reachable()) {
          calc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
          loaded++;
        }
      }

      angleSlots.add(new AngleSlot(hoodAngle, calc));
      totalLoaded += loaded;

      Logger.recordOutput(
          "FireControl/LUT/Hood" + String.format("%.0f", hoodAngle) + "/LaunchAngle", launchAngle);
      Logger.recordOutput(
          "FireControl/LUT/Hood" + String.format("%.0f", hoodAngle) + "/Entries", loaded);
      Logger.recordOutput(
          "FireControl/LUT/Hood" + String.format("%.0f", hoodAngle) + "/MaxRangeM",
          lut.maxRangeM());
    }

    initialized = true;

    long elapsed = System.currentTimeMillis() - totalStart;
    Logger.recordOutput("FireControl/LUT/TotalEntries", totalLoaded);
    Logger.recordOutput("FireControl/LUT/AngleSlots", angleSlots.size());
    Logger.recordOutput("FireControl/LUT/TotalGenerationTimeMs", elapsed);
    System.out.printf(
        "[FireControl] %d hood angles × LUT generated: %d total entries, took %dms%n",
        angleSlots.size(), totalLoaded, elapsed);
  }

  /**
   * Calculate the firing solution for this cycle by running every hood angle's ShotCalculator and
   * returning the one with the lowest RPM among valid solutions.
   *
   * @param robotPose current field-relative pose from odometry
   * @param fieldVelocity field-relative chassis speeds
   * @param robotVelocity robot-relative chassis speeds
   * @param visionConfidence 0.0 to 1.0, how much we trust the pose estimate
   * @return the best firing solution across all hood angles
   */
  public FireControlResult calculate(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      ChassisSpeeds robotVelocity,
      double visionConfidence) {

    if (!initialized || angleSlots.isEmpty()) {
      return FireControlResult.INVALID;
    }

    Translation2d hubCenter = getHubCenter();
    Translation2d hubForward = getHubForward();

    // Log raw inputs for debugging
    double rawDistToHub = robotPose.getTranslation().getDistance(hubCenter);
    Logger.recordOutput("FireControl/Input/RawDistToHub", rawDistToHub);
    Logger.recordOutput("FireControl/Input/RobotX", robotPose.getX());
    Logger.recordOutput("FireControl/Input/RobotY", robotPose.getY());
    Logger.recordOutput("FireControl/Input/HubX", hubCenter.getX());
    Logger.recordOutput("FireControl/Input/HubY", hubCenter.getY());
    Logger.recordOutput("FireControl/Input/HubForwardX", hubForward.getX());

    ShotCalculator.ShotInputs inputs =
        new ShotCalculator.ShotInputs(
            robotPose, fieldVelocity, robotVelocity, hubCenter, hubForward, visionConfidence);

    // Sweep all hood angles. Among valid solutions with adequate confidence,
    // pick the one with the LOWEST RPM — that's the best shot geometry
    // (the angle that most naturally arcs into the target at this distance).
    // A 21° shot needing 6000 RPM at 1.5m will lose to a 33° shot at 2500 RPM.

    ShotCalculator.LaunchParameters bestResult = ShotCalculator.LaunchParameters.INVALID;
    double bestAngle = shooterConstants.kMinHoodAngle;
    double bestRPM = Double.MAX_VALUE;

    for (AngleSlot slot : angleSlots) {
      ShotCalculator.LaunchParameters result = slot.shotCalc.calculate(inputs);

      // Log each slot's result for debugging
      String tag = String.format("FireControl/Slot%.0f", slot.hoodAngleDeg);
      Logger.recordOutput(tag + "/Valid", result.isValid());
      Logger.recordOutput(tag + "/RPM", result.isValid() ? result.rpm() : 0);
      Logger.recordOutput(tag + "/Confidence", result.isValid() ? result.confidence() : 0);

      if (result.isValid()
          && result.confidence() >= MIN_SWEEP_CONFIDENCE
          && result.rpm() < bestRPM) {
        bestRPM = result.rpm();
        bestResult = result;
        bestAngle = slot.hoodAngleDeg;
      }
    }

    // Log the winner
    Logger.recordOutput("FireControl/Valid", bestResult.isValid());
    Logger.recordOutput("FireControl/Confidence", bestResult.confidence());
    Logger.recordOutput("FireControl/RPM", bestResult.rpm());
    Logger.recordOutput("FireControl/HoodAngleDeg", bestAngle);
    Logger.recordOutput("FireControl/TOF", bestResult.timeOfFlightSec());
    Logger.recordOutput("FireControl/SolvedDistanceM", bestResult.solvedDistanceM());
    Logger.recordOutput("FireControl/Iterations", bestResult.iterationsUsed());
    Logger.recordOutput("FireControl/WarmStart", bestResult.warmStartUsed());
    if (bestResult.isValid()) {
      Logger.recordOutput("FireControl/DriveAngleDeg", bestResult.driveAngle().getDegrees());
      Logger.recordOutput("FireControl/AngularVelFF", bestResult.driveAngularVelocityRadPerSec());
    }

    return new FireControlResult(bestResult, bestAngle);
  }

  /** Returns the alliance-appropriate hub center. */
  public static Translation2d getHubCenter() {
    var alliance = DriverStation.getAlliance();
    boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
    return isBlue ? BLUE_HUB_CENTER : RED_HUB_CENTER;
  }

  /** Returns the hub forward vector for the current alliance. */
  public static Translation2d getHubForward() {
    var alliance = DriverStation.getAlliance();
    boolean isBlue = alliance.isPresent() && alliance.get() == Alliance.Blue;
    return isBlue ? BLUE_HUB_FORWARD : RED_HUB_FORWARD;
  }

  /** Bump RPM offset on ALL angle slots — bind to copilot D-pad. */
  public void adjustOffset(double delta) {
    for (AngleSlot slot : angleSlots) {
      slot.shotCalc.adjustOffset(delta);
    }
    if (!angleSlots.isEmpty()) {
      Logger.recordOutput("FireControl/RPMOffset", angleSlots.get(0).shotCalc.getOffset());
    }
  }

  /** Reset RPM offset on all angle slots — call on mode transitions. */
  public void resetOffset() {
    for (AngleSlot slot : angleSlots) {
      slot.shotCalc.resetOffset();
    }
  }

  /** Reset the warm-start state on all slots — call after a pose reset. */
  public void resetWarmStart() {
    for (AngleSlot slot : angleSlots) {
      slot.shotCalc.resetWarmStart();
    }
  }

  public boolean isInitialized() {
    return initialized;
  }
}
