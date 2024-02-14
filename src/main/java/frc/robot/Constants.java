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

import edu.wpi.first.math.geometry.*;
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
    public static Mode getMode() {
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

  public class GyroConstants {
    public static final double GYRO_HEADING_OFFSET_DEGREES = 90;
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
    public static final double APPLIED_VOLTS = 12.0;
  }

  /** Unchanging Values for the Under the Bumper Intake */
  public static class UTBIntakeConstants {
    public static final int UTB_INTAKE_CANID = 13;
    public static final int GEAR_RATIO = 2;
  }
}
