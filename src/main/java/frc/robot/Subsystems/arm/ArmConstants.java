// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

public final class ArmConstants {
  // PID Constants for the Actuator
  // TODO: Finalize PID values once they are tuned/determined + add 'final'
  // modifiers

  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 1.0;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static double KI = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static double KD = 0.0;
  /**
   * Updates the range of error acceptable from setpoint (The position of the Actuator can be within
   * 1% of the setpoint)
   */
  public static final double TOLERANCE_PERCENT = 0.01;

  // Sim Constants for the Arm
  /** Moment of Inertia for the Arm sim */
  public static final double MOI_KG_M2 = 0.00005; // TODO: update

  public static final boolean IS_SIMULATING_GRAVITY = false;

  // Real Constants
  /** The CAN ID of the Arm so it can be Identified on the CAN bus */
  public static final int CAN_ID = 20; // TODO: Update real CAN ID (on actual SPARK MAX)

  /** The Current Limit for the Actuator in Amps */
  public static final int CUR_LIM_A = 30;

  /**
   * The Gear Ratio of the Actuator (Controls Speed vs Power, Calculated from Teeth of Gears for
   * Control)
   */
  public static final double GEAR_RATIO = 99.1736;

  /** Length from pivot to wrist */
  public static final double LENGTH_M = 0.4126308486; // TODO: update

  /** The minimum angle the arm can rotate to */
  public static final double MIN_ANGLE_RAD = 0.390258413271767; // TODO: update

  /** The maximum angle the arm can rotate to */
  public static final double MAX_ANGLE_RAD = 1.8675; // TODO: update

  /** The angle where the arm starts */
  public static final double STARTING_ANGLE_RAD = 0.39025841327; // TODO: update

  /** Sets if Arm Motor is Inverted */
  public static final boolean IS_INVERTED = false;
}
