package frc.robot.Subsystems.wrist;

public final class WristConstants {

  // PID Constants for the Wrist
  /** Represents the proportional constant, multiplied by the current error */
  public static final double KP = 1.2;

  /** Represents the integral constant, multiplied by the total error */
  public static final double KI = 0.0;

  /** Represents the derivative constant, multiplied by the change in error */
  public static final double KD = 0.0;

  /** the position of the Wrist can be within 1% of the setpoint */
  public static final double TOLERANCE_PERCENT = 0.01;

  // Sim Constants
  /** Moment of inertia for the Wrist */
  public static final double MOI_KG_M2 = 0.000271862238;

  /** if the Wrist simulation is simulating */
  public static final boolean IS_SIMULATING_GRAVITY = false;

  // Real constants
  /** CAN ID of the Wrist motor */
  public static final int CAN_ID = 20;

  /** current limiting for Wrist */
  public static final int CUR_LIM_A = 30;

  /** Sets the inversion status of the Wrist motor */
  public static final boolean IS_INVERTED = true;

  /** Gear ratio of the Wrist motor */
  public static final double GEAR_RATIO = 58.33;

  /** Length from the Wrist */
  public static final double LENGTH_M = 0.4126308486;
  /** Minimum angle of the Wrist */
  public static final double MIN_ANGLE_RAD = 0.390258413271767;
  /** Maximum angle of the Wrist */
  public static final double MAX_ANGLE_RAD = 1.8675;

  /** Starting angle of the Wrist for sim, same as minimum angle */
  public static final double STARTING_ANGLE_RAD = MIN_ANGLE_RAD;

  public static final double ABS_ENCODER_OFFSET_RADS = -0.85;

  // Teleop Automations
  /** Amp Score Backside angle */
  public static final double AMP_BACKSIDE_DEG = 77;
  /** Amp Score Frontside angle */
  public static final double AMP_FRONTSIDE_RAD = 1.363;
}
