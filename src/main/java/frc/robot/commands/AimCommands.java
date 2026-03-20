package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FireControl;
import frc.robot.subsystems.shooter.FireControl.FireControlResult;
import frc.robot.subsystems.shooter.shooter;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AimCommands extends Command {

  private final shooter m_shooter;
  private final FireControl m_fireControl;
  private final Supplier<Pose2d> m_poseSupplier;
  private final Supplier<ChassisSpeeds> m_fieldVelocitySupplier;
  private final Supplier<ChassisSpeeds> m_robotVelocitySupplier;

  private int invalidCounter;
  private boolean currentlyOn;
  private FireControlResult m_lastValid = FireControlResult.INVALID;

  /** Minimum confidence (0-100) to allow a shot. */
  private static final double MIN_CONFIDENCE = 40.0;

  /** Cycles to hold the last-valid solution before stopping (prevents flickering). */
  private static final int HOLD_CYCLES = 10;

  public AimCommands(
      shooter shooter,
      FireControl fireControl,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldVelocitySupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier) {
    m_shooter = shooter;
    m_fireControl = fireControl;
    m_poseSupplier = poseSupplier;
    m_fieldVelocitySupplier = fieldVelocitySupplier;
    m_robotVelocitySupplier = robotVelocitySupplier;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    invalidCounter = 0;
    currentlyOn = false;
    m_lastValid = FireControlResult.INVALID;
  }

  @Override
  public void execute() {
    FireControlResult result =
        m_fireControl.calculate(
            m_poseSupplier.get(),
            m_fieldVelocitySupplier.get(),
            m_robotVelocitySupplier.get(),
            0.9); // vision confidence

    var shot = result.launchParams;

    Logger.recordOutput("Aim/Valid", shot.isValid());
    Logger.recordOutput("Aim/Confidence", shot.confidence());
    Logger.recordOutput("Aim/HoodAngleDeg", result.hoodAngleDeg);
    Logger.recordOutput("Aim/CurrentlyOn", currentlyOn);
    Logger.recordOutput("Aim/InvalidCounter", invalidCounter);

    if (shot.isValid() && shot.confidence() > MIN_CONFIDENCE) {
      // Good solution — set hood angle and spin up flywheel
      m_shooter.setHoodPosition(result.hoodAngleDeg);
      m_shooter.startFlywheel(shot.rpm());
      invalidCounter = 0;
      currentlyOn = true;
      m_lastValid = result;
    } else if (currentlyOn && m_lastValid.launchParams.isValid()) {
      // Solution dropped out — hold the last good values for a few cycles to prevent flicker
      invalidCounter++;
      if (invalidCounter >= HOLD_CYCLES) {
        currentlyOn = false;
        m_shooter.stopFlywheel();
      } else {
        m_shooter.setHoodPosition(m_lastValid.hoodAngleDeg);
        m_shooter.startFlywheel(m_lastValid.launchParams.rpm());
      }
    } else {
      // No valid solution — stop
      m_shooter.stopFlywheel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFlywheel();
    invalidCounter = 0;
    currentlyOn = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
