package frc.robot.Subsystems.shooter;

public class ShooterConstants {

  // PID Constants  TODO: Tune, update, finalize
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static final double TOP_KP = 0.75;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static final double TOP_KI = 0.25;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static final double TOP_KD = 0.0;
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static final double BOTTOM_KP = 0.75;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static final double BOTTOM_KI = 0.25;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static final double BOTTOM_KD = 0.0;
  /** Allows the RPM to be within 1% of the setpoint */
  public static final double TOLERANCE_PERCENT = 0.10;
  /** Max RPM achievable by the Shooter motors */
  public static final double MAX_RPM = 6350.0;

  // Sim constants
  /** Moment of Inertia for the shooter motors */
  public static final double MOI_KG_M2 = 0.0016007389;

  // Real constants
  /** Gear ratio of 1:1 for the shooter */
  public static final double GEAR_RATIO = 1.0;
  // Motor IDs TODO: Update?
  public static final int TOP_MOTOR_ID = 22;
  public static final int BOTTOM_MOTOR_ID = 23;
  /** Current limit Amps */
  public static final double CUR_LIM_A = 60;

  /** Auto-aiming shooting */
  public static final double[][] LOOKUP_TABLE_X_M_VS_THETA_DEG = {
    {1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6}, // x in meters
    {70, 60, 10, 5, 2.5, 1.25, .675, .3375, .1682, .05, 0.005}, // theta_max
    {50, 45, 30, 15, 7.5, 3.75, 675, .3375, .1682, .05, 0.005} // theta_min
  }; // random values

  // Inverted motors
  /** Sets the top motor to spin in the opposite direction of the Bottom Shooter Motor */
  public static final boolean TOP_MOTOR_IS_INVERTED = true;
  /** Sets the bottom motor to not be inverted and will therefore spin in a CW direction */
  public static final boolean BOTTOM_MOTOR_IS_INVERTED = true;
}
