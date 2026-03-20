package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.intake;
import frc.robot.subsystems.shooter.indexer;
import frc.robot.subsystems.shooter.shooter;

public class AutoCommands {
  private AutoCommands() {
    // Prevent instantiation
  }

  /**
   * Scores fuel without shimmy. Deploys intake, spins up flywheel, then feeds balls.
   *
   * @param indexer the indexer subsystem
   * @param shooter the shooter subsystem
   * @param intake the intake subsystem (must be deployed to shoot)
   */
  public static Command ScoreFuel(indexer indexer, shooter shooter, intake intake) {
    return ScoreFuel(indexer, shooter, intake, null, false);
  }

  /**
   * Scores fuel with optional shimmy to help feed balls from hopper.
   *
   * @param indexer the indexer subsystem
   * @param shooter the shooter subsystem
   * @param intake the intake subsystem (must be deployed to shoot)
   * @param drive the drive subsystem (can be null if shimmy is disabled)
   * @param enableShimmy whether to rotate robot back and forth to help feed balls
   */
  public static Command ScoreFuel(
      indexer indexer, shooter shooter, intake intake, Drive drive, boolean enableShimmy) {

    // Shimmy command: rotate back and forth to help feed balls
    Command shimmyCommand =
        enableShimmy && drive != null
            ? Commands.repeatingSequence(
                Commands.run(
                        () -> drive.runVelocity(new ChassisSpeeds(0, 0, 1.5)), // Rotate right
                        drive)
                    .withTimeout(0.15),
                Commands.run(
                        () -> drive.runVelocity(new ChassisSpeeds(0, 0, -1.5)), // Rotate left
                        drive)
                    .withTimeout(0.15))
            : Commands.none();

    return Commands.sequence(
            // Deploy intake first (required for shooter safety interlock)
            Commands.runOnce(() -> intake.setPivotAngle(100)),
            Commands.waitSeconds(0.3),
            // Start flywheel and wait for it to spin up
            Commands.runOnce(shooter::startFlywheel, shooter),
            Commands.waitSeconds(0.5),
            // Start indexer to feed balls with optional shimmy running in parallel
            Commands.runOnce(indexer::startIndexer),
            Commands.parallel(shimmyCommand, Commands.waitSeconds(8.0)),
            // Stop all systems
            Commands.runOnce(indexer::stopIndexer),
            Commands.runOnce(shooter::stopFlywheel, shooter),
            Commands.runOnce(
                () -> {
                  if (drive != null) {
                    drive.stop();
                  }
                }))
        .withName("Score Fuel");
  }
}
