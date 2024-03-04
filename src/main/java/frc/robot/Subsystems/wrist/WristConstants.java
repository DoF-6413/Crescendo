package frc.robot.Subsystems.wrist;

public final class WristConstants {
  // PID Constants for the wrist; TODO: finalize PID values once they are determined + add 'final'
  // modifiers

  /** represents the proportional constant, multiplied by the current error */
  public static final double KP = 1.2;

  /** represents the integral constant, multiplied by the total error */
  public static final double KI = 0.0;

  /** represents the derivative constant, multiplied by the change in error */
  public static final double KD = 0.0;

  /** the position of the wrist can be within 1% of the setpoint */
  public static final double TOLERANCE_PERCENT = 0.01;

  // Sim Constants
  /** moment of inertia for the wrist */
  public static final double MOI_KG_M2 = 0.000271862238;

  /** if the wrist simulation is simulating */
  public static final boolean IS_SIMULATING_GRAVITY = false; // TODO: UPDATE

  // Real constants
  /** CAN ID of the wrist motor */
  public static final int CAN_ID = 20;

  /** current limiting for wrist */
  public static final int CUR_LIM_A = 30;

  /** Gear ratio of the wrist motor */
  public static final double GEAR_RATIO = 58.33;
  /** length from the wrist */
  public static final double LENGTH_M = 0.4126308486;
  /** minimum and maximum angle of the physical wrist in radians */
  public static final double MIN_ANGLE_RAD = 0.390258413271767;

  public static final double MAX_ANGLE_RAD = 1.8675;
  /** starting angle of the wrist for sim, same as minimum angle */
  public static final double STARTING_ANGLE_RAD = MIN_ANGLE_RAD;

  public static final double ABS_ENCODER_OFFSET_RADS = -0.7;
}
