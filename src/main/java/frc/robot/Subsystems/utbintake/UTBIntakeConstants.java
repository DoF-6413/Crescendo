package frc.robot.Subsystems.utbintake;

public final class UTBIntakeConstants {
  // PID Constants for the UTB Intake
  // TODO: finalize PID values once they are determined + add 'final' modifiers
 
  /** Represents the proportional constant, multiplied by the current error */
  public static double KP = 0.0;

  /** Represents the integral constant, multiplied by the total error */
  public static double KI = 0.0;

  /** Represents the derivative constant, multiplied by the change in error */
  public static double KD = 0.0;

  /** The RPM of the UTB Intake can be within 1% of the setpoint */
  public static final double TOLERANCE_PERCENT = 0.01;

  /** Max RPM for PID */
  public static final int MAX_RPM = 2800;

  // Sim Constants
  /** Moment of inertia for the UTB Intake */
  public static final double MOI_KG_M2 = 0.0001929765;

  // Real Constants
  /** CAN ID of the UTB Intake motor */
  public static final int CAN_ID = 14;
  /** Current limiting for the UTB Intake */
  public static final int CUR_LIM_A = 30; // TODO: Verify
  /** Gear ratio of the UTB Intake */
  public static final double GEAR_RATIO = 2.0;
  /** Sets the inversion status of the UTB Intake motor */
  public static final boolean IS_INVERTED = false;
}
