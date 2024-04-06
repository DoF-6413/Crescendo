package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.util.Units;

public final class WristConstants {
  // Profiled PID Constants for the Wrist
  // TODO: tune & finalize

  /** Represents the proportional constant, multiplied by the current error */
  public static double KP = 1.2; // 1.15
  /** Represents the integral constant, multiplied by the total error */
  public static double KI = 0.0;
  /** Represents the derivative constant, multiplied by the change in error */
  public static double KD = 0.0; // 0.15
  /** */
  /** */
  public static double MAX_ACCELERATION = 0.0;

  // FF Constants for the Wrist
  // TODO: update and finalize
  /** */
  public static double KS = 0.0;
  /** */
  public static double KV = 0.0;
  /** */
  public static double KA = 0.0;

  /** The position of the Wrist can be within 2 degree of the setpoint */
  public static final double ANGLE_TOLERANCE = Units.degreesToRadians(0.5);

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

  public static final double MAX_VELOCITY = (Units.degreesToRotations(245.65) / GEAR_RATIO);

  /** Length from the Wrist */
  public static final double LENGTH_M = 0.4126308486;
  /** Minimum angle of the Wrist */
  public static final double MIN_ANGLE_RAD = 0.0;
  /** Maximum angle of the Wrist */
  public static final double MAX_ANGLE_RAD = 1.8675;

  /** Starting angle of the Wrist for sim, same as minimum angle */
  public static final double STARTING_ANGLE_RAD = MIN_ANGLE_RAD;

  public static final double ABS_ENCODER_OFFSET_RADS = -2.01715;

  // Teleop Automations
  /** AMP Score back side */
  public static final double AMP_BACK_SIDE_RAD = 1.368;
  /** AMP Score front side */
  public static final double AMP_FRONT_SIDE_RAD = 0.944;
  /** SOURCE Pickup back side */
  public static final double SOURCE_BACK_SIDE_RAD = 2.544;
  /** SPEAKER shot from podium */
  public static final double PODIUM_RAD = Units.degreesToRadians(-0.5);
  /** SPEAKER shot from subwoofer */
  public static final double SUBWOOFER_RAD = 0.612;

  public static final double DEFAULT_POSITION_DEG = 7;
}
