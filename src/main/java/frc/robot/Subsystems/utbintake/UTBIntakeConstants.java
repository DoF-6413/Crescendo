package frc.robot.Subsystems.utbintake;

public final class UTBIntakeConstants {
  // PID Constants for the wrist; TODO: finalize PID values once they are determined + add 'final'
  // modifiers

  /** represents the proportional constant, multiplied by the current error */
  public static double KP = 0.0;

  /** represents the integral constant, multiplied by the total error */
  public static double KI = 0.0;

  /** represents the derivative constant, multiplied by the change in error */
  public static double KD = 0.0;

  /** the RPM of the UTB intake can be within 1% of the setpoint */
  public static final double TOLERANCE_PERCENT = 0.01;

  /** max RPM for PID */
  public static final int MAX_RPM = 2000;

  // Sim Constants
  /** moment of inertia for the UTB intake */
  public static final double MOI_KG_M2 = 0.0001929765;

  // Real Constants
  /** CAN ID of the UTB intake motor */
  public static final int CAN_ID = 14; // TODO: Update
  /** current limiting for UTB intake */
  public static final int CUR_LIM_A = 30; // TODO: Update
  /** gear ratio of the UTB intake */
  public static final double GEAR_RATIO = 2;
  /** is brake mode */
  public static final boolean IS_BRAKE_MODE_ENABLED = true;
  /** is inverted for UTB motor */
  public static final boolean IS_INVERTED = false;
}
