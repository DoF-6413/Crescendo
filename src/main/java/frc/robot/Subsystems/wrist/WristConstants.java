package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.util.Units;

public final class WristConstants {
  // Profiled PID Constants for the Wrist
  /** Represents the proportional constant, multiplied by the current error */
  public static double KP = 0.8;
  /** Represents the integral constant, multiplied by the total error */
  public static double KI = 0.0;
  /** Represents the derivative constant, multiplied by the change in error */
  public static double KD = 0.01;
  /** The max velocity the Wrist can run at */
  public static double MAX_VELOCITY = 506.8427;
  /** The max acceleration the Wrist can run at */
  public static double MAX_ACCELERATION = 0.0;

  // Feedforward Constants for the Wrist
  /** KS represents the voltage added to overcome static friction */
  public static double KS = 0.0;
  /** KV represents the velocity gain */
  public static double KV = 0.0001;
  /** KV represents the acceleration gain */
  public static double KA = 0.0;

  /** The position of the Wrist can be within 1.5 degree of the setpoint */
  public static final double ANGLE_TOLERANCE = Units.degreesToRadians(1.5);

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

  /** Offset of the Arm absolute encoder */
  public static final double ABS_ENCODER_OFFSET_RADS = -2.01715;

  // Wrist positions
  /** Wrists default position for intaking NOTEs best */
  public static final double DEFAULT_POSITION_RAD = Units.degreesToRadians(9);
  /** AMP Score back side */
  public static final double AMP_BACK_SIDE_RAD = Units.degreesToRadians(72.38062);
  /** AMP Score front side */
  public static final double AMP_FRONT_SIDE_RAD = 0.944;
  /** SOURCE Pickup back side */
  public static final double SOURCE_BACK_SIDE_RAD = Units.degreesToRadians(140.7604);
  /** SPEAKER shot from subwoofer */
  public static final double SUBWOOFER_RAD = Units.degreesToRadians(32);
  /** SPEAKER shot from PODIUM */
  public static final double PODIUM_RAD = Units.degreesToRadians(14.3886);
  /** SPEAKER shot configuration from the PODIUM to score over "billboard-bots" */
  public static final double OVERSHOT_RAD = Units.degreesToRadians(38);
}
