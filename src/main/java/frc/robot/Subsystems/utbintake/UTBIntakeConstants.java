package frc.robot.Subsystems.utbintake;

public final class UTBIntakeConstants {
  // PID Constants
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 1.0;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static final double KI = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static final double KD = 0.0;
  /** The range the RPM of the UTB Intake can be within to be considered at it setpoint */
  public static final double RPM_TOLERENCE = 0.0;

  // Sim Constants
  /** Moment of inertia for the UTB Intake */
  public static final double MOI_KG_M2 = 0.0001929765; // TODO: Update

  // Real Constants
  /** CAN ID of the Top UTB Intake motor */
  public static final int TOP_CAN_ID = 14;
  /** CAN ID of the Bottom UTB Intake motor */
  public static final int BOTTOM_CAN_ID = 17;
  /** Current limiting for the UTB Intake */
  public static final int CUR_LIM_A = 30;
  /** Gear ratio of 30:24 for the top UTB Intake roller */
  public static final double GEAR_RATIO_TOP = 30 / 24;
  /** Gear ratio of 24:22 for the bottom UTB Intake roller */
  public static final double GEAR_RATIO_BOTTOM = 22 / 24;
  /** Sets the inversion status of the Top UTB Intake motor */
  public static final boolean IS_TOP_INVERTED = false;
  /** Sets the inversion status of the Bottom UTB Intake motor */
  public static final boolean IS_BOTTOM_INVERTED = false;
  /** Speed of the UTB Intake when intaking */
  public static final double INTAKE_PERCENT_SPEED = -0.75;
  /** Speed of the UTB Intake when outtaking */
  public static final double OUTTAKE_PERCENT_SPEED = 0.75;
}
