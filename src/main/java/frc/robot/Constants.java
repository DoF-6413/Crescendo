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
import edu.wpi.first.wpilibj.RobotBase;
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

  public static class OTBIntakeConstants {
    public static final int OTB_INTAKE_CANID = 0; // TODO: update this Id value please !!!
    public static final double OTB_GEAR_RATIO = 2.0;
    public static final int OTB_SMART_CURRENT_LIMIT_AMPS = 40; // TODO: Update

    // PID Constants for the OTB Intake Rollers
    // TODO: Finalize PID values once they are tuned/determined
    public static double OTB_INTAKE_KP = 0.0;
    public static double OTB_INTAKE_KI = 0.0;
    public static double OTB_INTAKE_KD = 0.0;
    public static final double OTB_INTAKE_TOLERANCE =
        0.01; // The RPM of the OTB Intake can be within 1% of the setpoint

    // Sim constants for the OTB Intake Rollers
    /** The moment of inertia for the OTB Intake Sim */
    public static final double OTB_MOI_KG_M2 = 0.0000023411;
  }

  public static class ActuatorConstants {
    public static final int ACTUATOR_CANID = 0; // TODO: update this id
    public static final double ACTUATOR_GEAR_RATIO = 193.75;
    public static final double ACTUATOR_MAX_ANGLE_RADS = Math.atan(-7.432 / 8.253) + (2 * Math.PI);
    public static final double ACTUATOR_MIN_ANGLE_RADS = Math.atan(11.105 / .096);
    public static final double ACTUATOR_START_ANGLE_RADS = ACTUATOR_MIN_ANGLE_RADS;
    /** Length from pivot to roller */
    public static final double ACTUATOR_LENGTH_M = Units.inchesToMeters(30.354561);
    
    public static final int ACTUATOR_SMART_CURRENT_LIMIT_AMPS = 40; // TODO: Update

    // PID Constants for the Actuator
    public static double ACTUATOR_KP =
        0.0; // TODO: Finalize PID values once they are tuned/determined
    public static double ACTUATOR_KI = 0.0;
    public static double ACTUATOR_KD = 0.0;
    public static final double ACTUATOR_TOLERANCE =
        0.05; // The position of the Actuator can be within 5% of the setpoint

    // Sim constants for the Actuator
    /** The moment of inertia for the Actuator Sim */
    public static final double ACTUATOR_MOI_KG_M2 = 0.0000453591;

    public static final boolean ACTUATOR_IS_SIMULATE_GRAVITY = true;
  }

  public class ArmConstants {
    public static final double MOTOR_GEAR_RATIO = 123; // TODO: update
    public static final double WRIST_APPLIED_VOLTS = 12;

    public static final double MOTOR_LENGTH = 0.4126308486;
    public static final double MOTOR_MIN_ANGLE = 0.390258413271767;
    public static final double MOTOR_MAX_ANGLE = 1.8675;
    public static final double MOTOR_STARTING_ANGLE = 0.39025841327;
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
}
