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

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
  }

  /** Field Measurements */
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
    public static final double KP = 5.0;
    public static final double KD = 100.0;
  }

  /** Wrist to SPEAKER alignment lookup table */
  public static class ShootingInterpolationConstants {

    /** Distance from the center subwoofer to the wall */
    public static final double SPEAKER_TO_WALL_M = 0.904;

    /**
     * Creates a table with distance from the SPEAKER as the x variable and the Wrist angle as the
     * output
     */
    public static final double[][] LOOKUP_TABLE_X_M_VS_THETA_DEG = {
      // from tests
      {
        0.5 + SPEAKER_TO_WALL_M,
        1 + SPEAKER_TO_WALL_M,
        1.5 + SPEAKER_TO_WALL_M,
        2 + SPEAKER_TO_WALL_M,
        2.5 + SPEAKER_TO_WALL_M,
        3 + SPEAKER_TO_WALL_M
      }, // x in meters
      /* Angles tested and collected 8-24-2024 */
      {31, 21, 15.5, 11, 8, 2}, // theta_max_degrees
      {23, 15, 10.5, 8, 4.25, 0} // theta_min_degrees
    };
  }

  /** Beam Break DIO Ports */
  public static class BeamBreakConstants {
    public static final int SHOOTER_BEAM_BREAK_PORT = 0;
  }

  /** Constants used for various commands */
  public static class CommandConstants {
    // Intakes Run
    /** Runs the Intake(s) to intake NOTEs into the robot */
    public static final boolean INTAKE_INWARDS = true;
    /** Runs the Intake(s) to eject NOTEs out of the robot */
    public static final boolean INTAKE_OUTWARDS = false;
    /** Starts the Intake(s) */
    public static final boolean RUN_INTAKE = false;
    /** Stops the Intake(s) */
    public static final boolean STOP_INTAKE = true;

    // Vision Pick Up
    public static final double VISION_PICKUP_TIMEOUT_SEC = 1.5; // TODO: Test and Update

    // Feeder Reverse
    public static final double FEEDER_REVERSE_TIMEOUT_SEC = 3; // TODO: Test and Update

    // SPEAKER Rotation Target Override
    public static final boolean SPEAKER_ROTATION_OVERRIDE_ENABLE = true;
    public static final boolean SPEAKER_ROTATION_OVERRIDE_DISABLE = false;

    // NOTE Rotation Target Override
    public static final boolean NOTE_ROTATION_OVERRIDE_ENABLE = true;
    public static final boolean NOTE_ROTATION_OVERRIDE_DISABLE = false;

    // Preload Shot
    public static final double PRELOAD_SHOT_TIMEOUT_SEC = 2.0;
  }

  /** Constants for all Vision systems */
  public final class VisionConstants {
    /** Offsets the back left camera's position to the center of the robot */
    public static final Transform3d LEFT_CAMERA_ROBOT_OFFSET =
        new Transform3d(
            new Translation3d(-Units.inchesToMeters(10.541), Units.inchesToMeters(11.695), 0),
            new Rotation3d(Math.PI, 0, Math.PI - Units.degreesToRadians(10.881)));

    /** Offsets the back right camera's position to the center of the robot */
    public static final Transform3d RIGHT_CAMERA_ROBOT_OFFSET =
        new Transform3d(
            new Translation3d(-Units.inchesToMeters(10.541), -Units.inchesToMeters(11.695), 0),
            new Rotation3d(Math.PI, 0, Math.PI + Units.degreesToRadians(14.881)));

    /** The name of the Lime Light camera */
    public static final String LIME_LIGHT_NAME = "limelight";

    // Photon Camera names
    public static final String LEFT_CAMERA_NAME = "Back_Left";
    public static final String RIGHT_CAMERA_NAME = "Back_Right";

    /**
     * The range a NOTE is allowed to be within to stop the robot from rotating during NOTE
     * alignment
     */
    public static final double LL_NOTE_RANGE = 10;
  }

  /** Contants for PathPlanner and Path Finding */
  public static class PathPlannerConstants {
    public static final double TRANSLATION_KP = 1.2;
    public static final double TRANSLATION_KD = 0.2;
    public static final double ROTATION_KP = 0.3125;
    public static final double ROTATION_KD = 0.025;

    // PathFinding
    /**
     * Max translational and rotational speed and acceleration used for PathPlanner's PathFinding
     */
    public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
        new PathConstraints(3, 3, Units.degreesToRadians(515.65), Units.degreesToRadians(262.82));
    /** Position to align the robot with the AMP in the Blue Wing */
    public static final Pose2d AMP_BLUE_END_POSE =
        new Pose2d(1.85, 7.69, new Rotation2d(Units.degreesToRadians(-90)));
    /** Position to align the robot with the AMP in the Red Wing */
    public static final Pose2d AMP_RED_END_POSE =
        new Pose2d(14.69, 7.69, new Rotation2d(Units.degreesToRadians(-90)));
    /** Start position at the AMP side of the Subwoofer */
    public static final Pose2d SUB_AMP_START_POSE = new Pose2d(0.80, 6.59, Rotation2d.fromDegrees(60));
  }
}
