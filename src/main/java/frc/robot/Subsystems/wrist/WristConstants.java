package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.util.Units;

public final class WristConstants {
  // PID Constants for the Wrist

  /** Represents the proportional constant, multiplied by the current error */
  public static final double KP = 1.2;

  /** Represents the integral constant, multiplied by the total error */
  public static final double KI = 0.0;

  /** Represents the derivative constant, multiplied by the change in error */
  public static final double KD = 0.0;

  /** The position of the Wrist can be within 1 degree of the setpoint */
  public static final double ANGLE_TOLERANCE = Units.degreesToRadians(1);

  // Sim Constants
  /** Moment of inertia for the Wrist */
  public static final double MOI_KG_M2 = 0.000271862238;

  /** If the Wrist simulation is simulating gravity */
  public static final boolean IS_SIMULATING_GRAVITY = false;

  // Real constants
  /** CAN ID of the Wrist motor */
  public static final int CAN_ID = 20;

  /** Current limiting for Wrist */
  public static final int CUR_LIM_A = 30;

  /** Sets the inversion status of the Wrist motor */
  public static final boolean IS_INVERTED = true;

  /** Gear ratio of the Wrist motor */
  public static final double GEAR_RATIO = 58.33;

  /** Length from the Wrist */
  public static final double LENGTH_M = 0.4126308486;
  /** Minimum angle of the Wrist */
  public static final double MIN_ANGLE_RAD = 0.0;
  /** Maximum angle of the Wrist */
  public static final double MAX_ANGLE_RAD = 1.8675;

  /** Starting angle of the Wrist for sim, same as minimum angle */
  public static final double STARTING_ANGLE_RAD = MIN_ANGLE_RAD;

  public static final double ABS_ENCODER_OFFSET_RADS = -0.85;

  // Teleop Automations
  /** AMP Score back side */
  public static final double AMP_BACK_SIDE_DEG = 77;

  public static final double AMP_BACK_SIDE_RAD = Units.degreesToRadians(77);
  /** AMP Score front side */
  public static final double AMP_FRONT_SIDE_RAD = 1.363;
  /** SOURCE Pickup back side */
  public static final double SOURCE_BACK_SIDE_RAD = Units.degreesToRadians(155.0);
}
