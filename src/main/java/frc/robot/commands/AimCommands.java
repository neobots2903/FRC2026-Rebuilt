package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterKinematics;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.shooter.shooterConstants;
import java.util.function.Supplier;

public class AimCommands extends Command {

  private final shooter m_shooter;
  private final Supplier<Pose2d> m_robotPose;

  public AimCommands(shooter shooter, Supplier<Pose2d> robotPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_robotPose = robotPose;
  }

  @Override
  public void initialize() {
    // Code to run when the command is initially scheduled.
  }

  @Override
  public void execute() {
    // Code to run repeatedly while the command is scheduled.
    double optimalHoodAngleDeg =
        ShooterKinematics.getOptimalHoodAngleDeg(
            m_robotPose.get(), shooterConstants.TARGET_POSE_METERS.toTranslation2d());
    m_shooter.setHoodPosition(optimalHoodAngleDeg);
    m_shooter.startFlywheel();
  }

  @Override
  public void end(boolean interrupted) {
    // Code to run once after the command ends or is interrupted.
    m_shooter.stopFlywheel();
  }

  @Override
  public boolean isFinished() {
    // Return true when the command should end.
    return false; // Change this condition as needed.
  }
}
