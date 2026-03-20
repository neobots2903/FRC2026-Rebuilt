package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class shooter extends SubsystemBase {
  private final shooterIO io;
  private final shooterIOInputsAutoLogged inputs = new shooterIOInputsAutoLogged();

  public shooter(shooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
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
    Logger.recordOutput("Shooter/Hood/HoodPositionDegrees", inputs.hoodPositionDegrees);
    Logger.recordOutput("Shooter/Hood/HoodAppliedVolts", inputs.hoodAppliedVolts);
  }

  // Controls the shooter:

  // Starts shooter with RPM from constants
  public void startFlywheel() {
    io.setFlywheelVelocity(shooterConstants.ShooterRPM);
  }

  // Starts shooter with specified RPM
  public void startFlywheel(double velocityRPM) {
    io.setFlywheelVelocity(velocityRPM);
  }

  // Stops shooter
  public void stopFlywheel() {
    io.stop();
  }

  public double getFlywheelVelocity() {
    return inputs.flywheelVelocity;
  }

  public boolean isFlywheelRunning() {
    if (inputs.flywheelSetpointRPM == 0.0) {
      return false;
    } else {
      return true;
    }
  }

  public void setHoodPosition(double positionDegrees) {
    io.setHoodPosition(positionDegrees);
  }

  public double getHoodPosition() {
    return inputs.hoodPositionDegrees;
  }
}
