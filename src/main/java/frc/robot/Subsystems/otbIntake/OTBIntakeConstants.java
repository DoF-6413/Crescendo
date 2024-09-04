package frc.robot.Subsystems.otbIntake;

public class OTBIntakeConstants {
  // Sim constants
  /** The moment of inertia for the OTB Intake Sim */
  public static final double MOI_KG_M2 = 0.0000023411;

  // Real Constants
  /** The CAN ID of the OTB Intake so it can be Identified on the CAN bus */
  public static final int CAN_ID = 16;
  /** Gear ratio of 2:1 for the OTB Intake */
  public static final double GEAR_RATIO = 2.0;
  /** The Current Limit for the OTB Intake in Amps */
  public static final int CUR_LIM_A = 40; // TODO: Update
  /** Ensure the OTB Intake wont be inverted on startup */
  public static final boolean IS_INVERTED = false;
  /** Max RPM achievable by the OTB Intake */
  public static final double MAX_RPM = 1400.0; // TODO: Verify, maxValue based on sim
  /** Disables break mode */
  public static final boolean IS_BRAKE_MODE_ENABLED = false;
}
