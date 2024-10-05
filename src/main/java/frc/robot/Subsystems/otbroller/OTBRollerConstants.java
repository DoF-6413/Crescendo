// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbroller;

/** Add your docs here. */
public class OTBRollerConstants {
  // PID Constants
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 1.0;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static double KI = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static double KD = 0.0;
  /** The range the RPM of the OTB Rollers can be within to be considered at it setpoint */
  public static final double RPM_TOLERENCE = 0.0;

  // Sim Constants
  /** Moment of Interia of the OTB Roller. Used for the flywheel physics simulation */
  public static final double MOI_KG_M2 = 0.0001; // TODO: Update

  // Real Constants
  /** The CAN ID of the OTB Roller so it can be Identified on the CAN bus */
  public static final int CAN_ID = 16;
  /** Gear ratio of 30:24 for the OTB Roller */
  public static final double GEAR_RATIO = 0.80;
  /** The Current Limit for the OTB Roller in Amps */
  public static final int CUR_LIM_A = 40; // TODO: Verify
  /** Ensure the OTB Roller wont be inverted on startup */
  public static final boolean IS_INVERTED = false; // TODO: Verify
  /** Percent speed for intaking */
  public static final double INTAKE_PERCENT_SPEED = -0.30;
  /** Percent speed for outtaking */
  public static final double OUTTAKE_PERCENT_SPEED = 0.30;
}
