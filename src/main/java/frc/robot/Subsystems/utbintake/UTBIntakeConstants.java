package frc.robot.Subsystems.utbintake;

public final class UTBIntakeConstants {
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
