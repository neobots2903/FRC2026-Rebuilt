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
    Logger.recordOutput("Shooter/FlywheelVelocity", inputs.flywheelVelocity);
    Logger.recordOutput("Shooter/FlywheelCurrent", inputs.flywheelCurrent);
    Logger.recordOutput("Shooter/FlywheelAppliedVolts", inputs.flywheelAppliedVolts);
    Logger.recordOutput("Shooter/FlywheelSetpointRPM", inputs.flywheelSetpointRPM);
    Logger.recordOutput("Shooter/HoodPositionDegrees", inputs.hoodPositionDegrees);
    Logger.recordOutput("Shooter/HoodAppliedCurrentAmps", inputs.hoodAppliedCurrentAmps);
    Logger.recordOutput("Shooter/HoodAppliedVolts", inputs.hoodAppliedVolts);
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
    io.setFlywheelVelocity(0.0);
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
}
