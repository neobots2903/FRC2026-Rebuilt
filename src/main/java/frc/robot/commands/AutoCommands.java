package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.indexer;
import frc.robot.subsystems.shooter.shooter;

public class AutoCommands {
  private AutoCommands() {
    // Prevent instantiation
  }

  public static Command ScoreFuel(indexer indexer, shooter shooter) {
    return Commands.sequence(
            Commands.runOnce(
                    () -> {
                      indexer.startIndexer();
                    })
                .andThen(Commands.waitSeconds(0.2)),
            Commands.run(shooter::startFlywheel, shooter).andThen(Commands.waitSeconds(8.0)),
            Commands.runOnce(
                () -> {
                  indexer.stopIndexer();
                }),
            Commands.run(shooter::stopFlywheel, shooter))
        .withName("Score Fuel");
  }
}
