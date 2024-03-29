// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.math.util.Units;

/** Actuator Constants */
public final class ActuatorConstants {
  // PID Constants for the Actuator

  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static final double KP = 1.0;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static final double KI = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static final double KD = 0.0;
  /**
   * Updates the range of error acceptable from setpoint (The position of the Actuator can be within
   * 5% of the setpoint)
   */
  public static final double ANGLE_TOLERANCE = Units.degreesToRadians(2);

  // Sim Constants for the Actuator

  /** The Moment of Inertia for the Actuator Sim */
  public static final double MOI_KG_M2 = 0.0000453591;

  /** Declares if Actuator is Simulating Gravity */
  public static final boolean IS_SIMULATING_GRAVITY = false;

  // Real Constants for the Actuator
  /** The CAN ID of the Actuator so it can be Identified on the CAN bus */
  public static final int CAN_ID = 15;
  /**
   * The Gear Ratio of the Actuator (Controls Speed vs Power, Calculated from Teeth of Gears for
   * Control)
   */
  public static final double GEAR_RATIO = 193.75;

  /** The Current Limit for the Actuator in Amps */
  public static final int CUR_LIM_A = 20;

  /** The maximum angle the actuator can rotate to */
  public static final double MAX_ANGLE_RADS = Units.degreesToRadians(135);
  // Math.atan(-7.432 / 8.253) + (2 * Math.PI); // 5.6

  /** The minimum angle the actuator can rotate to */
  public static final double MIN_ANGLE_RADS = 0;
  // Math.atan(11.105 / .096); // 1?

  /** The angle where the actuator starts */
  public static final double START_ANGLE_RADS = MIN_ANGLE_RADS;

  /** Length from pivot to roller */
  public static final double LENGTH_M = Units.inchesToMeters(30.354561);

  /** Sets if Actuator Motor is Inverted */
  public static final boolean IS_INVERTED = true;
}
