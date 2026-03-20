// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.intake;
import frc.robot.subsystems.intake.intakeIO;
import frc.robot.subsystems.intake.intakeIOsim;
import frc.robot.subsystems.intake.intakeIOsparkmax;
import frc.robot.subsystems.shooter.ShooterKinematics;
import frc.robot.subsystems.shooter.indexer;
import frc.robot.subsystems.shooter.indexerIO;
import frc.robot.subsystems.shooter.indexerIOreal;
import frc.robot.subsystems.shooter.indexerIOsim;
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
  // private final climb climb;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));

        // Real robot, instantiate hardware IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOPhotonVision(camera0Name, robotToCamera0));
        // vision =
        // new Vision(
        // demoDrive::addVisionMeasurement,
        // new VisionIOPhotonVision(camera0Name, robotToCamera0),
        // new VisionIOPhotonVision(camera1Name, robotToCamera1));

        intake = new intake(new intakeIOsparkmax());

        shooter = new shooter(new shooterIOreal());

        indexer = new indexer(new indexerIOreal());

        // climb = new climb(new climbIOsparkmax());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // Sim robot, instantiate physics sim IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        intake = new intake(new intakeIOsim());

        shooter = new shooter(new shooterIOsim());

        indexer = new indexer(new indexerIOsim());

        // climb = new climb(new climbIOsim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        intake = new intake(new intakeIO() {});
        shooter = new shooter(new shooterIO() {});
        indexer = new indexer(new indexerIO() {});
        // climb = new climb(new climbIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Example of adding an auto routine to path planner and auto chooser
    // NamedCommands.registerCommand(camera0Name, getAutonomousCommand());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Drive inputs:

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
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

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Intake inputs:

    operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (!intake.isIntakeRunning()) {
                    intake.startIntake();
                  } else if (intake.isIntakeRunning()) {
                    intake.stopIntake();
                  }
                }));

    operatorController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (!intake.isIntakeRunning()) {
                    intake.startIntakeIntake();
                  } else if (intake.isIntakeRunning()) {
                    intake.stopIntake();
                  }
                }));

    operatorController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (intake.getPivotAngle() > 50) {
                    intake.setPivotAngle(0);
                  } else if (intake.getPivotAngle() < 50) {
                    intake.setPivotAngle(100);
                  }
                }));

    // Shooter inputs:

    operatorController
        .x()
        .whileTrue(Commands.run(shooter::startFlywheel, shooter))
        .onFalse(Commands.run(shooter::stopFlywheel, shooter));

    operatorController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (shooter.getHoodPosition() > shooterConstants.kMinHoodAngle) {
                    shooter.setHoodPosition(shooter.getHoodPosition() - 5);
                  }
                }));

    operatorController
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (shooter.getHoodPosition() < shooterConstants.kMaxHoodAngle) {
                    shooter.setHoodPosition(shooter.getHoodPosition() + 5);
                  }
                }));

    operatorController
        .y()
        .whileTrue(
            Commands.parallel(
                new AimCommands(shooter, drive::getPose),
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> ShooterKinematics.calculateTurretAngleRotation(drive.getPose()))));

    operatorController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (!indexer.isIndexerRunning()) {
                    indexer.startIndexer();
                  } else if (indexer.isIndexerRunning()) {
                    indexer.stopIndexer();
                  }
                }));

    // Climb inputs:

    // operatorController
    //     .rightBumper()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               // if (climb.getClimbAngle() < 180) {
    //               climb.setClimbAngle(100);
    //               // } else if (climb.getClimbAngle() > 180) {
    //               //   climb.setClimbAngle(0);
    //               // }
    //             }));

    // TODO:
    // Run this in parallel with 'joystickDriveAtAngle' to point the robot at the hub while
    // shooting.
    // You can probably get the angle to point towards using your turret angle code from
    // ShooterKinematics since it's just an angle (needs to be modified slightly)
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
