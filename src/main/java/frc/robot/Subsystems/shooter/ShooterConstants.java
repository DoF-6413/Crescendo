package frc.robot.Subsystems.shooter;

public class ShooterConstants {

  // PID & FF Constants
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 0.0025;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static double KI = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static double KD = 0.00002;

  /** KS represents the voltage added to overcome static friction */
  public static double KS = 0.0;
  /** KS represents the */
  public static double KV = 0.0225;
  /** KS represents the */
  public static double KA = 0.8;

  /** Allows the RPM to be within % of the setpoint */
  public static final int RPM_TOLERANCE = 200;
  /** Max RPM achievable by the Shooter motors */
  public static final double MAX_RPM = 6350.0;

  // Sim constants
  /** Moment of Inertia for the shooter motors */
  public static final double MOI_KG_M2 = 0.0016007389;

  // Real constants
  /** Gear ratio of 1:1 for the shooter */
  public static final double GEAR_RATIO = 1.0;
  // Motor IDs
  public static final int TOP_MOTOR_ID = 22;
  public static final int BOTTOM_MOTOR_ID = 23;
  /** Current limit Amps */
  public static final double CUR_LIM_A = 60;
  /** Enables current limiting for TalonFX/Falcon500 motors */
  public static final boolean ENABLE_CUR_LIM = true;

  // Inverted motors
  /** Sets the inversion status of the Top Shooter motor */
  public static final boolean TOP_MOTOR_IS_INVERTED = true;
  /** Sets the inversion status of the Bottom Shooter motor */
  public static final boolean BOTTOM_MOTOR_IS_INVERTED = true;

  // Teleop Automations
  /** AMP score speed */
  public static final double AMP_RPM = 2500;
  /**
   * Shooter speed that will keep the STEM Center alive in case the Wrist angle is off when testing
   * shots :)
   */
  public static final double SLOW_RPM = 1000;
  /** SPEAKER score speed at or near the subwoofer */
  public static final double CLOSE_RPM = 4000;
  /** SPEAKER score speed from mid range */
  public static final double MID_RANGE_RPM = 5000;
  /** SPEAKER score speed from mid stage and beyond */
  public static final double FAR_RPM = 6000;
  /** Shooter speed for sending NOTEs from the SOURCE to Midfield */
  public static final double SOURCE_FEED_RPM = 5500;
  /** Shooter speed for sending NOTES from Midfield to alliance WING */
  public static final double MIDFIELD_FEEDING_RPM = 3000;
}
