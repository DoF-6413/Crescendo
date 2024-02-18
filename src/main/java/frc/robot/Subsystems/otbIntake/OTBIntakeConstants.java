package frc.robot.Subsystems.otbIntake;

public class OTBIntakeConstants {
  // PID Constants  TODO: Tune, update, finalize
  public static double KP = 0.0;
  public static double KI = 0.0;
  public static double KD = 0.0;
  /** The RPM of the OTB Intake can be within 1% of the setpoint */
  public static final double TOLERANCE_PERCENT = 0.01;

  // Sim constants
  /** The moment of inertia for the OTB Intake Sim */
  public static final double MOI_KG_M2 = 0.0000023411;

  // Real Constants
  public static final int CAN_ID = 0; // TODO: Update
  public static final double GEAR_RATIO = 2.0;
  public static final int CURR_LIM_A = 40; // TODO: Update
  public static final boolean IS_INVERTED = false;
  public static final double MAX_VALUE = 1400.0; // TODO: Verify, maxValue based on sim
  public static final boolean IS_BRAKE_MODE_ENABLED = false; // TODO: Update/Decide
}
