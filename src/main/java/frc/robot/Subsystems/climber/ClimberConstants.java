// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.climber;

import edu.wpi.first.math.util.Units;

public final class ClimberConstants {
  // PID Constants for the Actuator
  // TODO: Finalize PID values once they are tuned/determined
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double LEFT_KP = 0.0;

  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static double LEFT_KI = 0.0;

  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static double LEFT_KD = 0.0;

  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double RIGHT_KP = 0.0;

  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static double RIGHT_KI = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static double RIGHT_KD = 0.0;

  /**
   * Updates the range of error acceptable from setpoint (The position of the Climbers can be within
   * 1% of the setpoint)
   */
  public static final double TOLERANCE_PERCENT = 0.01;

  // Sim Constants for the Climber
  public static final boolean SIMULATE_GRAVITY = false;

  // Real Constants for the Climber
  /** The CAN ID of the left Arm motor so it can be Identified on the CAN bus */
  public static final int LEFT_CAN_ID = 0; // TODO: Update
  /** The CAN ID of the right Arm motor so it can be Identified on the CAN bus */
  public static final int RIGHT_CAN_ID = 0; // TODO: Update

  // TODO: Confirm that one of the motors will need to be inverted
  /** Ensures that the Left Climber Motor will not be inverted upon start up */
  public static final boolean LEFT_IS_INVERTED = false;

  /** Ensures that the Right Climber Motor will be inverted upon start up */
  public static final boolean RIGHT_IS_INVERTED = true;

  /**
   * The Gear Ratio of the Actuator (Controls Speed vs Power, Calculated from Teeth of Gears for
   * Control)
   */
  public static final double GEAR_RATIO = 80; // 80:1 Gear Ratio

  /**
   * The Mass of the End of the "Elevator" or the Climber (This is litterally just the mass of hook)
   */
  public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(0.095);

  /** The Radius of the Spool that the "Elevator" or Climber rotates around */
  public static final double DRUM_RADIUS_M = Units.inchesToMeters(2);

  /** The minimum angle the climber can extend to from the lowest point of the climber */
  public static final double MIN_HEIGHT_M = Units.inchesToMeters(2.969);

  /** The maximum height the climber can extend to from the lowest point of the climber */
  public static final double MAX_HEIGHT_M = Units.inchesToMeters(40.12448);

  /** The position the climber starts at */
  public static final double STARTING_HEIGHT_M = MIN_HEIGHT_M;
}
