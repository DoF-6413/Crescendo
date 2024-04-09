package frc.robot.Subsystems.feeder;

public class FeederConstants {

  // PID Constants  TODO: Tune, update, finalize
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 0.0;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static double KI = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static double KD = 0.0;
  /** Allows the RPM to be within 1% of the setpoint */
  public static final double TOLERANCE_PERCENT = 0.01;
  /** Max RPM achievable by the Feeder motor */
  public static final double MAX_RPM = 4800.0; // TODO: Update

  // Sim constants
  public static final double MOI_KG_M2 = 0.0001; // TODO: Update

  // Real constants
  /** CAN ID for the feeder motor */
  public static final int FEEDER_MOTOR_ID = 21;
  /** Gear ratio of 4:3 for the Feeder */
  public static final double GEAR_RATIO = 1.33; // TODO: Update
  /** Current limit Amps */
  public static final double CUR_LIM_A = 40; // TODO: Update
  /** Used to set the Feeder motor to not be inverted on startup */
  public static final boolean IS_INVERTED = false;

  // Teleop Automations
  /** AMP score speed */
  public static final double AMP_RPM = 1500;
  /** SOURCE pickup speed */
  public static final double SOURCE_RPM = 1500;
  /** SPEAKER score speed */
  public static final double SPEAKER_RPM = 1500;
}
