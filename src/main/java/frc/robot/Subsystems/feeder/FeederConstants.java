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
  /** Max RPM achievable by the Indexer motor*/
  public static final double MAX_VALUE = 0.0; //TODO: Update

  // Real constants
  /** Gear ratio of 4:3 for the Indexer */
  public static final double GEAR_RATIO = 4/3; // TODO: Update
  /** Current limit Amps */
  public static final double CUR_LIM_A = 40; // TODO: Update
  /** Used to set the Indexer motor to not be inverted on startup */
  public static final boolean IS_INVERTED = false;
}
