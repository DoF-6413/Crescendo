// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Defines State of Robot */
  public static class RobotStateConstants {
    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }

    /** Gets Robot Mode (Real, Sim, or Replay) */
    public static final Mode getMode() {
      if (RobotBase.isReal()) {
        return Mode.REAL;
      } else if (RobotBase.isSimulation()) {
        return Mode.SIM;
      } else {
        return Mode.REPLAY;
      }
    }

    /** Get Alliance (Blue, Red, Null) */
    public static final Optional<Alliance> getAlliance() {
      return DriverStation.getAlliance();
    }

    /** If CAN takes too long, it cancels */
    public static final int CAN_CONFIG_TIMEOUT_SEC = 30;

    /** Command Loop Seconds */
    public static final double LOOP_PERIODIC_SEC = 0.02;

    /** Average Battery Voltage */
    public static final double BATTERY_VOLTAGE = 12;
  }

  /** Controller Ports */
  public static class OperatorConstants {
    public static final int DRIVE_CONTROLLER = 0;
    public static final int AUX_CONTROLLER = 1;
    public static final int DEV_CONTROLLER = 2;
  }

  public final class FieldConstants {
    // all in meters
    public static final double BLUE_SPEAKER_X = 0.23;
    public static final double RED_SPEAKER_X = 16.49;
    public static final double SPEAKER_Y = 5.3; // 16.412;
    public static final double SPEAKER_Z = 2.08; // height of opening
    public static final Translation2d BLUE_SPEAKER = new Translation2d(0, 5.55);
    public static final Translation2d RED_SPEAKER = new Translation2d(16.58, 5.55);
  }

  public static class HeadingControllerConstants {
    public static final double HEADING_CONTROLLER_KP = 0.0;
    public static final double HEADING_CONTROLLER_KD = 0.0;
  }

  public static class ShootingInterpolationConstants {
    /** Auto-aiming shooting */
    public static final double SPEAKER_TO_WALL_M = 0.904;

    public static final double[][] LOOKUP_TABLE_X_M_VS_THETA_DEG = {
      // from tests
      {
        0.5 + SPEAKER_TO_WALL_M,
        1 + SPEAKER_TO_WALL_M,
        1.5 + SPEAKER_TO_WALL_M,
        2 + SPEAKER_TO_WALL_M,
        2.5 + SPEAKER_TO_WALL_M,
        3 + SPEAKER_TO_WALL_M,
        3.5 + SPEAKER_TO_WALL_M,
        4 + SPEAKER_TO_WALL_M,
        4.5 + SPEAKER_TO_WALL_M,
        5 + SPEAKER_TO_WALL_M
      }, // x in meters
      {32, 20, 13, 9, 4, 2, -2, -4, -4, -6}, // theta_max_degrees
      {24, 15, 8, 5, 2, -1, -3, -4, -4, -6} // theta_min_degrees
      // {34, 22, 15, 11, 6, 4, 0, -2, -3, -5}, // theta_max_degrees
      // {26, 17, 10, 7, 4, 1, -1, -2, -3, -5} // theta_min_degrees
    };
  }
}
