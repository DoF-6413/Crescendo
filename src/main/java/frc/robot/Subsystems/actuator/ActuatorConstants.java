// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.math.util.Units;

/** Actuator Constants */
public final class ActuatorConstants {
    /** The CAN ID of the Actuator so it can be Identified on the CAN bus */
    public static final int CAN_ID = 0; // TODO: update this id
    /**
     * The Gear Ratio of the Actuator (Controls Speed vs Power, Calculated from
     * Teeth of Gears for Control)
     */
    public static final double GEAR_RATIO = 193.75;
    /** The maximum angle the actuator can twist to */
    public static final double MAX_ANGLE_RADS = Math.atan(-7.432 / 8.253) + (2 * Math.PI);
    /** The minimum angle the actuator can twist to */
    public static final double MIN_ANGLE_RADS = Math.atan(11.105 / .096);
    /** The angle where the actuator starts */
    public static final double START_ANGLE_RADS = MIN_ANGLE_RADS;

    /** Length from pivot to roller */
    public static final double LENGTH_M = Units.inchesToMeters(30.354561);

    /** The Current Limit for the Actuator in Amps */
    public static final int CUR_LIM_A = 40; // TODO: Update

    // PID Constants for the Actuator
    // TODO: Finalize PID values once they are tuned/determined + add 'final'
    // modifiers
    /**
     * KP represents the constant multiplied by the current error from setpoint
     * (Proportional Error)
     */
    public static double KP = 0.0;
    /**
     * KI represents the constant multiplied by the total error from setpoint
     * (Integrated Error)
     */
    public static double KI = 0.0;
    /**
     * KD represents the constant multiplied by the velocity error from setpoint
     * (Derived Error)
     */
    public static double KD = 0.0;
    /**
     * Updates the range of error acceptable from setpoint (The position of the
     * Actuator can be within 5% of the setpoint)
     */
    public static final double TOLERANCE_PERCENT = 0.05;

    // Sim constants for the Actuator

    /** The Moment of Inertia for the Actuator Sim */
    public static final double MOI_KG_M2 = 0.0000453591;

    /** Declares if Actuator is Simulating Gravity */
    public static final boolean IS_SIMULATING_GRAVITY = true;
}