// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.util.Units;

public final class ArmConstants {
    public static final int CAN_ID = 0; // TODO: update
    public static final double GEAR_RATIO = 99.1736;
    public static final double MOI_KG_M2 = 0.00005; // TODO: update
    public static final double LENGTH_M = 0.4126308486; // TODO: double check units
    public static final double MIN_ANGLE_RAD = 0.390258413271767; // TODO: update
    public static final double MAX_ANGLE_RAD = 1.8675; // TODO: update
    public static final double STARTING_ANGLE_RAD = 0.39025841327; // TODO: update
    public static final boolean IS_SIMULATING_GRAVITY = false;
    public static final int CUR_LIM_A = 30;

    // PID Constants for the Actuator
    // TODO: Finalize PID values once they are tuned/determined + add 'final'
    // modifiers
    public static double ARM_KP = 0.0;
    public static double ARM_KI = 0.0;
    public static double ARM_KD = 0.0;
    public static final double ARM_TOLERANCE_PERCENT = 0.01;
}