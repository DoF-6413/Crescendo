// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.climber;

import edu.wpi.first.math.util.Units;

public final class ClimberConstants {
  public static final double CLIMBER_TOLERANCE = 0.01;
  public static final int LEFT_CLIMBER_CANID = 0; // TODO: Update
  public static final int RIGHT_CLIMBER_CANID = 0; // TODO: Update

  // TODO: Confirm that one of the motors will need to be inverted
  public static final boolean LEFT_CLIMBER_INVERTED =
      false; // Ensures that the Left Climber Motor will not be inverted upon start up
  public static final boolean RIGHT_CLIMBER_INVERTED =
      true; // Ensures that the Right Climber Motor will be inverted upon start up

  public static final double CLIMBER_GEAR_RATIO = 80; // 80:1 Gear Ratio

  // Sim Constants
  public static final double CLIMBER_CARRIAGE_MASS_KG = Units.lbsToKilograms(0.095);
  public static final double CLIMBER_DRUM_RADIUS_M = Units.inchesToMeters(2);
  public static final double CLIMBER_MIN_HEIGHT_M = Units.inchesToMeters(10.845);
  public static final double CLIMBER_MAX_HEIGHT_M = Units.inchesToMeters(48);
  public static final double CLIMBER_STARTING_HEIGHT_M = CLIMBER_MIN_HEIGHT_M;
  public static final boolean CLIMBER_SIMULATE_GRAVITY = false;

  public static double LEFT_CLIMBER_KP = 0.0;
  public static double LEFT_CLIMBER_KI = 0.0;
  public static double LEFT_CLIMBER_KD = 0.0;

  public static double RIGHT_CLIMBER_KP = 0.0;
  public static double RIGHT_CLIMBER_KI = 0.0;
  public static double RIGHT_CLIMBER_KD = 0.0;
}
