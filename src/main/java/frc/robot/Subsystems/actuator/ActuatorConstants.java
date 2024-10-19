// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.math.util.Units;

/** Actuator Constants */
public final class ActuatorConstants {
  // PID Constants

  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static final double KP = 1.0;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static final double KI = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static final double KD = 0.0;
  /**
   * The acceptable range the position of the Actuator can be within to be considered at the
   * setpoint
   */
  public static final double ANGLE_TOLERANCE = Units.degreesToRadians(2);

  // Sim Constants

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
  /** Sets if Actuator Motor is Inverted */
  public static final boolean IS_INVERTED = true;
  /** The maximum angle the Actuator can rotate to */
  public static final double MAX_ANGLE_RADS = Units.degreesToRadians(135);
  /** The minimum angle the Actuator can rotate to */
  public static final double MIN_ANGLE_RADS = 0;
  /** The angle where the actuator starts */
  public static final double START_ANGLE_RADS = MIN_ANGLE_RADS;
  /** Length from pivot to roller */
  public static final double LENGTH_M = Units.inchesToMeters(30.354561);
}
