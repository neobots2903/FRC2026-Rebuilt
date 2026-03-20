// Copyright (c) 2021-2026 Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.indexer;
import frc.robot.subsystems.indexer.indexerIO;
import frc.robot.subsystems.indexer.indexerIOreal;
import frc.robot.subsystems.indexer.indexerIOsim;
import frc.robot.subsystems.intake.intake;
import frc.robot.subsystems.intake.intakeIO;
import frc.robot.subsystems.intake.intakeIOsim;
import frc.robot.subsystems.intake.intakeIOsparkmax;
import frc.robot.subsystems.shooter.FireControl;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.shooter.shooterConstants;
import frc.robot.subsystems.shooter.shooterIO;
import frc.robot.subsystems.shooter.shooterIOreal;
import frc.robot.subsystems.shooter.shooterIOsim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final intake intake;
  private final Drive drive;
  private final Vision vision;
  private final shooter shooter;
  private final indexer indexer;
  private final FireControl fireControl;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOPhotonVision(camera0Name, robotToCamera0));

        intake = new intake(new intakeIOsparkmax());

        shooter = new shooter(new shooterIOreal());

        indexer = new indexer(new indexerIOreal());

        fireControl = new FireControl();
        fireControl.initialize();

        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        intake = new intake(new intakeIOsim());

        shooter = new shooter(new shooterIOsim());

        indexer = new indexer(new indexerIOsim());

        fireControl = new FireControl();
        fireControl.initialize();

        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        intake = new intake(new intakeIO() {});
        shooter = new shooter(new shooterIO() {});
        indexer = new indexer(new indexerIO() {});
        fireControl = new FireControl();
        fireControl.initialize();
        break;
    }

    // Register named commands for PathPlanner autos
    // Use shimmy version to help feed balls from hopper
    NamedCommands.registerCommand(
        "Score Fuel", AutoCommands.ScoreFuel(indexer, shooter, intake, drive, false));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    // Wire up safety interlocks - flywheel won't spin when intake is stowed
    shooter.setIntakeStowedSupplier(intake::isStowed);

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Configures button-to-command mappings for driver and operator controllers. */
  private void configureButtonBindings() {

    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0 degrees when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0 degrees when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // D-Pad snap to cardinal directions (relative to field)
    driverController
        .povUp()
        .onTrue(
            DriveCommands.snapToAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                Rotation2d.kZero));
    driverController
        .povRight()
        .onTrue(
            DriveCommands.snapToAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                Rotation2d.fromDegrees(270)));
    driverController
        .povDown()
        .onTrue(
            DriveCommands.snapToAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                Rotation2d.fromDegrees(180)));
    driverController
        .povLeft()
        .onTrue(
            DriveCommands.snapToAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                Rotation2d.fromDegrees(90)));

    // Slow mode when LT is held
    driverController
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() * 0.5,
                () -> -driverController.getLeftX() * 0.5,
                () -> -driverController.getRightX() * 0.5));

    //  When RB is held, intake out back at full speed
    driverController
        .rightBumper()
        .onTrue(Commands.run(() -> intake.shootOutBack(), intake))
        .onFalse(Commands.run(() -> intake.stopIntake(), intake));
  }

  private void configureOperatorBindings() {
    // A: Toggle intake pivot (up/down)
    operatorController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (intake.getPivotAngle() > 50) {
                    intake.setPivotAngle(0);
                  } else {
                    intake.setPivotAngle(100);
                  }
                }));

    // B: Toggle intake into robot
    operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (!intake.isIntakeRunning()) {
                    intake.startIntake();
                  } else {
                    intake.stopIntake();
                  }
                }));

    // X: Toggle indexer
    operatorController
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (!indexer.isIndexerRunning()) {
                    indexer.startIndexer();
                  } else {
                    indexer.stopIndexer();
                  }
                }));

    // Y: Auto-aim + shoot (hold)
    operatorController
        .y()
        .whileTrue(
            Commands.parallel(
                new AimCommands(
                    shooter,
                    fireControl,
                    drive::getPose,
                    drive::getFieldRelativeSpeeds,
                    drive::getChassisSpeeds),
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> {
                      // Aim the chassis at the hub while driving
                      var hub = FireControl.getHubCenter();
                      var pose = drive.getPose();
                      double dx = hub.getX() - pose.getX();
                      double dy = hub.getY() - pose.getY();
                      return new Rotation2d(dx, dy);
                    })));

    // RT (analog): Manual flywheel with proportional RPM (1800-3600 RPM)
    // Uses quadratic curve for finer control at lower trigger values
    operatorController
        .rightTrigger(0.1) // Small deadband
        .whileTrue(
            Commands.run(
                () -> {
                  double triggerValue = operatorController.getRightTriggerAxis();
                  // Apply input squaring for finer low-end control
                  double curved = triggerValue * triggerValue;
                  // Map 0-1 curved trigger to 1800-3600 RPM
                  double rpm = 1800 + (curved * 1800);
                  shooter.startFlywheel(rpm);
                },
                shooter))
        .onFalse(Commands.runOnce(() -> shooter.stopFlywheel(), shooter));

    // Left Stick Y: Hood angle (up/down, proportional)
    // This runs continuously as a default command for the shooter hood
    shooter.setDefaultCommand(
        Commands.run(
            () -> {
              double stickY = -operatorController.getLeftY(); // Invert so up = up
              if (Math.abs(stickY) > 0.1) { // Deadband
                double currentAngle = shooter.getHoodPosition();
                // Adjust hood by small increments based on stick position
                double adjustment = stickY * 1.0; // 1 degree per cycle at full deflection
                double newAngle = currentAngle + adjustment;
                // Clamp to valid range
                newAngle =
                    Math.max(
                        shooterConstants.kMinHoodAngle,
                        Math.min(shooterConstants.kMaxHoodAngle, newAngle));
                shooter.setHoodPosition(newAngle);
              }
            },
            shooter));

    // D-Pad: RPM trim controls
    operatorController.povUp().onTrue(Commands.runOnce(() -> fireControl.adjustOffset(25)));
    operatorController.povDown().onTrue(Commands.runOnce(() -> fireControl.adjustOffset(-25)));
    operatorController.povLeft().onTrue(Commands.runOnce(() -> fireControl.resetOffset()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
