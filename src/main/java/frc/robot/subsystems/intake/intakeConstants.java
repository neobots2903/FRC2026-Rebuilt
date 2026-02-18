package frc.robot.subsystems.intake;

public class intakeConstants {
  // Motor IDs
  public static final int kPivotMotorId = 9;
  public static final int kIntakeMotorId = 10;
  // Positions (in degrees)
  public static final double kInPosition = 0.0;
  public static final double kOutPosition = 90.0;
  // Speeds
  public static final double kIntakeVoltage = 8.0;
  public static final double kEjectVoltage = -4.0;
  // Current limits
  public static final int kPivotCurrentLimit = 30;
  public static final int kIntakeCurrentLimit = 20;
  public static final double kIntakeStallCurrent = 15.0;
  // PID values
  public static final double kPivotP = 0.2;
  public static final double kPivotI = 0.0;
  public static final double kPivotD = 0.05;
  // Tolerances
  public static final double kPivotPositionTolerance = 2.0; // In degrees
  // Mechanical configurations
  public static final double kPivotGearRatio = 1.0;
  public static final double kIntakeGearRatio = 1.0;
  // Conversion factors
  public static final double kPivotDegreesPerRotation = 360.0 / kPivotGearRatio;
}
