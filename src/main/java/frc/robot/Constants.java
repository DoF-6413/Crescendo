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

import edu.wpi.first.math.geometry.*; // Rotation3d, Transform3d, Translation2d, Transation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*; // DriverStation and RobotBase
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public static final int CAN_CONFIG_TIMEOUT_SEC = 500;

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

  public class DriveConstants {
    /**
     * Gives the PID Constant P for the Drive Motors depending on whether the Module is an L3 or L2
     */
    public static final double driveKP(boolean isL3) {
      if (isL3) {
        return DRIVE_KP_KRAKEN;
      } else {
        return DRIVE_KP_NEO;
      }
    }

    /** Gives the PID Constant I for the Drive Motors depending on whether the Module is an L3 or */
    public static final double driveKI(boolean isL3) {
      if (isL3) {
        return DRIVE_KI_KRAKEN;
      } else {
        return DRIVE_KI_NEO;
      }
    }

    /**
     * Gives the PID Constant D for the Drive Motors depending on whether the Module is an L3 or L2
     */
    public static final double driveKD(boolean isL3) {
      if (isL3) {
        return DRIVE_KD_KRAKEN;
      } else {
        return DRIVE_KD_NEO;
      }
    }

    /**
     * Gives the FeedFoward Constant S for the Drive Motors depending on whether the Module is an
     * L3or L2
     */
    public static final double driveKS(boolean isL3) {
      if (isL3) {
        return DRIVE_KS_KRAKEN;
      } else {
        return DRIVE_KS_NEO;
      }
    }

    // ** Gives the Feed Forward V for the Drive Motors depending on whether the
    // Module is an L3 or L2 */
    public static final double driveKV(boolean isL3) {
      if (isL3) {
        return DRIVE_KV_KRAKEN;
      } else {
        return DRIVE_KV_NEO;
      }
    }

    // ** Gives the Gear Ratio for the Module depending on whether the Module is an
    // L3 or L2 */
    public static final double gearRatio(boolean isL3) {
      if (isL3) {
        return GEAR_RATIO_L3;
      } else {
        return GEAR_RATIO_L2;
      }
    }

    /** Wheel Radius in Meters */
    public static final double WHEEL_RADIUS_M = Units.inchesToMeters(2);

    /**
     * Chassis Length and Width (distance between the centerline of two adjacent wheels, same for x
     * & y bc DT is square)
     */
    public static final double TRACK_WIDTH_M = Units.inchesToMeters(32.173359);

    /** Max Speed the Robot Can Travel in One Linear Direction (m/s) */
    public static final double MAX_LINEAR_SPEED_M_PER_SEC = 4.5; // TODO: Update

    /**
     * Max Speed the Robot Can Rotate (rad/s) Angular Speed can be Calulated by Dividing Max Linear
     * Speed by Radius of the Circle an Object is Moving Around (v/r = w) The Radius of the Swerve
     * Drive is Equivelant to Half of the Distance of one Corner to the Other Corner This Can be
     * Calculated by Using Pythagoreans Theorem on Two of the Sides of the Robot and taking Half of
     * the Hypotenuses
     */
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC =
        MAX_LINEAR_SPEED_M_PER_SEC / (Math.sqrt(2 * (TRACK_WIDTH_M * TRACK_WIDTH_M)) / 2);

    // PID Constants for Neo Drive PID
    public static final double DRIVE_KP_NEO = 0.05; // TODO: Update
    public static final double DRIVE_KI_NEO = 0; // TODO: Update
    public static final double DRIVE_KD_NEO = 0; // TODO: Update

    // PID Constants for Kraken Drive PID
    public static final double DRIVE_KP_KRAKEN = 0; // TODO: Update
    public static final double DRIVE_KI_KRAKEN = 0; // TODO: Update
    public static final double DRIVE_KD_KRAKEN = 0; // TODO: Update

    // Feed Forward Constants for Kraken Drive
    public static final double DRIVE_KS_KRAKEN = 0; // TODO: Update
    public static final double DRIVE_KV_KRAKEN = 0; // TODO: Update

    // Feed Forward Constants for Neo Drive
    public static final double DRIVE_KS_NEO = 0.4; // TODO: Update
    public static final double DRIVE_KV_NEO = 0.4; // TODO: Update

    // PID Constants for Neo Steer PID
    public static final double STEER_KP_NEO = 7.0; // TODO: Update
    public static final double STEER_KI_NEO = 0; // TODO: Update
    public static final double STEER_KD_NEO = 0; // TODO: Update

    /** Gear Ratio for MK4I L3 */
    public static final double GEAR_RATIO_L3 = 6.12;

    /** Gear Ratio for MK4I L2 */
    public static final double GEAR_RATIO_L2 = 6.75;

    public static final boolean IS_BRAKE_MODE = true;

    /** Used in Robot Characterization Tool to Help Determine Drive Values like PID */
    public static final boolean IS_CHARACTERIZING = false;

    public static final double DRIVE_MOI_KG_M2 = 0.0003125; // moment of inertia for sim
    public static final double STEER_MOI_KG_M2 = 0.0003125; // TODO: Update

    public static final Translation2d[] getModuleTranslations() {
      return new Translation2d[] {
        new Translation2d(DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0),
        new Translation2d(DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0),
        new Translation2d(-DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0),
        new Translation2d(-DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0)
      };
    }
  }

  public class WristConstants {
    public static final double WRIST_GEAR_RATIO = 58.33; 
    public static final double WRIST_LENGTH = 0.4126308486; 
    public static final double WRIST_MIN_ANGLE = 0.390258413271767;
    public static final double WRIST_MAX_ANGLE = 1.8675;
    public static final double WRIST_STARTING_ANGLE = 0.39025841327;
  }

  public class VisionConstants {

    public static final Transform3d cameraOnRobotOffsets =
        new Transform3d(
            new Translation3d(0, 0, 0), // update this value
            new Rotation3d(0, 0, 0)); // update this offset value
  }

  public static class ShooterConstants {

    // Gear ratio of 1:1 for the prototype Horizontal-Rollers/Top-Bottom Shooter
    public static final double SHOOTER_GEAR_RATIO = 1.0;

    // Motor IDs
    public static final int TOP_SHOOTER_MOTOR_ID = 14; // TalonFX currently set to 14 TODO: Update?
    public static final int BOTTOM_SHOOTER_MOTOR_ID =
        15; // TalonFX currently set to 15 and is named "Climb motor" on the Phoenix tuner TODO:
    // Update?

    // Inverted motors
    public static final boolean TOP_SHOOTER_MOTOR_INVERTED =
        true; // Sets the top motor to spin in the opposite direction of the Bottom Shooter Motor
    public static final boolean BOTTOM_SHOOTER_MOTOR_INVERTED =
        false; // Sets the bottom motor to not be inverted and will therefore spin in a CW direction

    // PID Constants  TODO: Tune and update
    public static final double TOP_SHOOTER_KP =
        0.0; // The "P" value of the PID for the top shooter motor
    public static final double TOP_SHOOTER_KI =
        0.0; // The "I" value of the PID for the top shooter motor
    public static final double TOP_SHOOTER_KD =
        0.0; // The "D" value of the PID for the top shooter motor
    public static final double BOTTOM_SHOOTER_KP =
        0.0; // The "P" value of the PID for the bottom shooter motor
    public static final double BOTTOM_SHOOTER_KI =
        0.0; // The "I" value of the PID for the bottom shooter motor
    public static final double BOTTOM_SHOOTER_KD =
        0.0; // The "D" value of the PID for the bottom shooter motor

    // Current limit Amps
    public static final double SMART_CURRENT_LIMIT_AMPS = 60;

    // Flywheel simulation constants
    public static final double SHOOTER_MOI_KG_M2 =
        0.0016007389; // Moment of Inertia for the shooter motors
  }

  /** Unchanging Values for the Under the Bumper Intake */
  public static class UTBIntakeConstants {
    public static final int UTB_INTAKE_CANID = 0; // TODO: Update later
    public static final int GEAR_RATIO = 2; // 2:1 Gear Ratio
    // public static final double UTB_KP = 1;
    // public static final double UTB_KI = 1;
    // public static final double UTB_KD = 1;
    public static final double UTB_MOI_KG_M2 = 0.0001929765;
    public static final double UTB_INTAKE_TOLERANCE = 0.05;
    public static double UTB_INTAKE_KP = 0;
    public static double UTB_INTAKE_KI = 0;
    public static double UTB_INTAKE_KD = 0;
  }

  public static class ClimberConstants {
    public static final int LEFT_CLIMBER_CANID = 0; // TODO: Update
    public static final int RIGHT_CLIMBER_CANID = 0; // TODO: Update

    // TODO: Confirm that one of the motors will need to be inverted
    public static final boolean LEFT_CLIMBER_INVERTED =
        false; // Ensures that the Left Climber Motor will not be inverted upon start up
    public static final boolean RIGHT_CLIMBER_INVERTED =
        true; // Ensures that the Right Climber Motor will be inverted upon start up

    public static final double CLIMBER_GEAR_RATIO = 40; // 40:1 Gear Ratio

    // Sim Constants
    public static final double CLIMBER_CARRIAGE_MASS_KG = 5.0; // TODO: Update
    public static final double CLIMBER_DRUM_RADIUS_M = 0.25; // TODO: Update
    public static final double CLIMBER_MIN_HEIGHT_M = 0.2; // TODO: Update
    public static final double CLIMBER_MAX_HEIGHT_M = 2.0; // TODO: Update
    public static final double CLIMBER_STARTING_HEIGHT_M = 0.4; // TODO: Update
    public static final boolean CLIMBER_SIMULATE_GRAVITY = false; // TODO: Update
  }

  public class ArmConstants {
    public static final double MOTOR_GEAR_RATIO = 123; // TODO: update
    public static final double WRIST_APPLIED_VOLTS = 12;

    public static final double MOTOR_LENGTH = 0.4126308486;
    public static final double MOTOR_MIN_ANGLE = 0.390258413271767;
    public static final double MOTOR_MAX_ANGLE = 1.8675;
    public static final double MOTOR_STARTING_ANGLE = 0.39025841327;
  }
}
