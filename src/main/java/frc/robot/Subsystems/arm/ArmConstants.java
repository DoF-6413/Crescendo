// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.util.Units;

public final class ArmConstants {
  // Profiled PID & FF Constants for the arm

  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 1.0;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static double KI = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static double KD = 0.0;
  /** The max velocity the Arm can run at */
  public static double MAX_VELOCITY = 0.0;
  /** The max acceleration the Arm can run at */
  public static double MAX_ACCELERATION = 0.0;

  /** KS represents the voltage added to overcome static friction */
  public static double KS = 0.0;
  /** KV represents the velocity gain */
  public static double KV = 0.0;
  /** KV represents the acceleration gain */
  public static double KA = 0.0;

  /**
   * Updates the range of error acceptable from setpoint (The position of the arm can be within 3
   * degrees of the setpoint)
   */
  public static final double ANGLE_TOLERANCE = Units.degreesToRadians(3);

  // Sim Constants for the Arm
  /** Moment of Inertia for the Arm sim */
  public static final double MOI_KG_M2 = 0.00005; // TODO: update

  public static final boolean IS_SIMULATING_GRAVITY = false;

  // Real Constants
  /** The CAN ID of the Arm so it can be Identified on the CAN bus */
  public static final int CAN_ID = 19;

  /** The Current Limit for the arm in Amps */
  public static final int CUR_LIM_A = 30;

  /**
   * The Gear Ratio of the arm (Controls Speed vs Power, Calculated from Teeth of Gears for Control)
   */
  public static final double GEAR_RATIO = 99.1736;

  /** Length from pivot to arm */
  public static final double LENGTH_M = 0.4126308486; // TODO: update

  /** The minimum angle the arm can rotate to */
  public static final double MIN_ANGLE_RAD = 0.0; // TODO: update

  /** The maximum angle the arm can rotate to */
  public static final double MAX_ANGLE_RAD = 1.8675; // TODO: update

  /** The angle where the arm starts */
  public static final double STARTING_ANGLE_RAD = MIN_ANGLE_RAD;

  /** Sets if Arm Motor is Inverted */
  public static final boolean IS_INVERTED = true;

  /** Offsets the Arm to be 0 at its inital position */
  public static final double OFFSET_RAD = 1.3640066793392915;

  // Teleop Automations
  /** AMP Score back side */
  public static final double AMP_BACK_SIDE_RAD = 0.880;
  /** AMP Score front side */
  public static final double AMP_FRONT_SIDE_RAD = 2.244;
  /** SOURCE Pickup back side */
  public static final double SOURCE_BACK_SIDE_RAD = Units.degreesToRadians(28.53329);
  /** Subwoofer scoring angle */
  public static final double SUBWOOFER_RAD = Units.degreesToRadians(6);
  /** SPEAKER shot configuration from the PODIUM to score over "billboard-bots" */
  public static final double OVERSHOT_RAD = Units.degreesToRadians(83); // TODO: Update
}
