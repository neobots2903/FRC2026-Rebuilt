package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class shooter extends SubsystemBase {
  private final shooterIO io;
  private final shooterIOInputsAutoLogged inputs = new shooterIOInputsAutoLogged();
  private BooleanSupplier intakeStowedSupplier = () -> false; // Default: assume intake is deployed

  public shooter(shooterIO io) {
    this.io = io;
  }

  /**
   * Set a supplier that returns true when the intake is stowed/retracted. The flywheel will refuse
   * to spin when the intake is stowed as a safety measure.
   */
  public void setIntakeStowedSupplier(BooleanSupplier supplier) {
    this.intakeStowedSupplier = supplier;
  }

  /** Returns true if it's safe to run the flywheel (intake is not stowed). */
  public boolean isSafeToShoot() {
    return !intakeStowedSupplier.getAsBoolean();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Safety: Stop flywheel if intake becomes stowed while running
    if (intakeStowedSupplier.getAsBoolean() && inputs.flywheelSetpointRPM != 0) {
      io.setFlywheelVelocity(0);
    }

    Logger.processInputs("Shooter/Shooter", inputs);
    Logger.recordOutput("Shooter/Flywheel/FlywheelVelocity", inputs.flywheelVelocity);
    Logger.recordOutput("Shooter/Flywheel/Flywheel2Velocity", inputs.flywheel2Velocity);
    Logger.recordOutput("Shooter/Flywheel/FlywheelCurrent", inputs.flywheelCurrent);
    Logger.recordOutput("Shooter/Flywheel/Flywheel2Current", inputs.flywheel2Current);
    Logger.recordOutput("Shooter/Flywheel/FlywheelAppliedVolts", inputs.flywheelAppliedVolts);
    Logger.recordOutput("Shooter/Flywheel/Flywheel2AppliedVolts", inputs.flywheel2AppliedVolts);
    Logger.recordOutput("Shooter/Flywheel/FlywheelSetpointRPM", inputs.flywheelSetpointRPM);
    Logger.recordOutput("Shooter/Flywheel/Flywheel2SetpointRPM", inputs.flywheel2SetpointRPM);
    Logger.recordOutput("Shooter/Hood/HoodPositionDegrees", inputs.hoodPositionDegrees);
    Logger.recordOutput("Shooter/Hood/HoodAppliedCurrentAmps", inputs.hoodAppliedCurrentAmps);
    Logger.recordOutput("Shooter/Hood/HoodAppliedVolts", inputs.hoodAppliedVolts);
    Logger.recordOutput("Shooter/Hood/HoodSetPointDegrees", inputs.hoodSetPointDegrees);
    Logger.recordOutput("Shooter/SafeToShoot", isSafeToShoot());
  }

  // Starts shooter with RPM from constants (only when safe)
  public void startFlywheel() {
    if (isSafeToShoot()) {
      io.setFlywheelVelocity(shooterConstants.ShooterRPM);
    } else {
      io.setFlywheelVelocity(0);
    }
  }

  // Starts shooter with specified RPM (only when safe)
  public void startFlywheel(double velocityRPM) {
    if (isSafeToShoot()) {
      io.setFlywheelVelocity(velocityRPM);
    } else {
      io.setFlywheelVelocity(0);
    }
  }

  // Stops shooter
  public void stopFlywheel() {
    io.stop();
  }

  public double getFlywheelVelocity() {
    return inputs.flywheelVelocity;
  }

  public boolean isFlywheelRunning() {
    return inputs.flywheelSetpointRPM != 0.0;
  }

  public void setHoodPosition(double positionDegrees) {
    io.setHoodPosition(positionDegrees);
  }

  public double getHoodPosition() {
    return inputs.hoodPositionDegrees;
  }
}
