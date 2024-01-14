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

import edu.wpi.first.math.util.Units;
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
    public static final int CAN_CONFIG_TIMEOUT = 500;

    /** Command Loop Seconds */
    public static final double LOOP_PERIODIC_SEC = 0.02;
  }

  /** Controller Ports */
  public static class OperatorConstants {
    public static final int DRIVE_CONTROLLER = 0;
    public static final int AUX_CONTROLLER = 1;
  }

  public class DriveConstants {
    // Gives the PID Constant P for the Drive Motors depending on whether the Module is an L3 or L2
    public static final double driveKP(boolean isL3) {
      if (isL3) {
        return DRIVE_KP_KRAKEN;
      } else {
        return DRIVE_KP_NEO;
      }
    }

    // Gives the PID Constant I for the Drive Motors depending on whether the Module is an L3 or L2
    public static final double driveKI(boolean isL3) {
      if (isL3) {
        return DRIVE_KI_KRAKEN;
      } else {
        return DRIVE_KI_NEO;
      }
    }

    // Gives the PID Constant D for the Drive Motors depending on whether the Module is an L3 or L2

    public static final double driveKD(boolean isL3) {
      if (isL3) {
        return DRIVE_KD_KRAKEN;
      } else {
        return DRIVE_KD_NEO;
      }
    }

    // Gives the FeedFoward Constant S for the Drive Motors depending on whether the Module is an L3 or L2

    public static final double driveKS(boolean isL3) {
      if (isL3) {
        return DRIVE_KS_KRAKEN;
      } else {
        return DRIVE_KS_NEO;
      }
    }

    // Gives the Feed Forward V for the Drive Motors depending on whether the Module is an L3 or L2

    public static final double driveKV(boolean isL3) {
      if (isL3) {
        return DRIVE_KV_KRAKEN;
      } else {
        return DRIVE_KV_NEO;
      }
    }

    // Gives the Gear Ratio for the Module depending on whether the Module is an L3 or L2
    public static final double gearRatio(boolean isL3) {
      if (isL3) {
        return GEAR_RATIO_L3;
      } else {
        return GEAR_RATIO_L2;
      }
    }

    // wheel
    public static final double WHEEL_RADIUS_M = Units.inchesToMeters(1.5); //TODO: verify

    // chassis 
    public static final double TRACK_WIDTH = Units.inchesToMeters(32.173359); //distance between the centerline of two adjacent wheels, same for x & y bc DT is square

    // PID Constants for Neo Drive PID
    public static final double DRIVE_KP_NEO = 0;
    public static final double DRIVE_KI_NEO = 0;
    public static final double DRIVE_KD_NEO = 0;

    // PID Constants for Kraken Drive PID
    public static final double DRIVE_KP_KRAKEN = 0;
    public static final double DRIVE_KI_KRAKEN = 0;
    public static final double DRIVE_KD_KRAKEN = 0;

    // Feed Forward Constants for Kraken Drive
    public static final double DRIVE_KS_KRAKEN = 0;
    public static final double DRIVE_KV_KRAKEN = 0;

    // Feed Forward Constants for Neo Drive
    public static final double DRIVE_KS_NEO = 0;
    public static final double DRIVE_KV_NEO = 0;

    // PID Constants for Neo Steer PID
    public static final double STEER_KP_NEO = 0;
    public static final double STEER_KI_NEO = 0;
    public static final double STEER_KD_NEO = 0;

    // Gear Ratio for MK4I L3
    public static final double GEAR_RATIO_L3 = 0;
    // Gear Ratio for MK4I L2
    public static final double GEAR_RATIO_L2 = 0;

  }
}
