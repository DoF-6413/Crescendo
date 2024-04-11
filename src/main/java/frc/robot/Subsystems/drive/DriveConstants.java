package frc.robot.Subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.Optional;

public final class DriveConstants {

  // PID Constants for Kraken Drive
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double DRIVE_KP_KRAKEN = 0.0;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static double DRIVE_KI_KRAKEN = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static double DRIVE_KD_KRAKEN = 0.0;
  // Feed Forward Constants for Kraken Drive
  // Feed Forward values used in sim: S = 0.4, V = 0.4

  /** KS represents the voltage required to overcome static friction */
  public static double DRIVE_KS_KRAKEN = 0.115;

  /** KV represents the voltage used every second per meter */
  public static double DRIVE_KV_KRAKEN = 0.12978;

  // PID Constants for Steer (Neos)
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double STEER_KP_NEO = 6.4;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static double STEER_KI_NEO = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static double STEER_KD_NEO = 0.05;

  // PID Constants for Neo Drive
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static final double DRIVE_KP_NEO = 0.0;
  /** KI represents the constant multiplied by the total error from setpoint (Integrated Error) */
  public static final double DRIVE_KI_NEO = 0.0;
  /** KD represents the constant multiplied by the velocity error from setpoint (Derived Error) */
  public static final double DRIVE_KD_NEO = 0.0;

  // Feed Forward Constants for Neo Drive
  /** KS represents the voltage required to overcome static friction */
  public static final double DRIVE_KS_NEO = 0.4;
  /** KV represents the voltage used every second per meter */
  public static final double DRIVE_KV_NEO = 0.4;

  /**
   * returns P of PID constants for Drive Motors depending on whether the Module is an L3 (kraken)
   * or L2 (neo)
   */
  public static final double driveKP(Optional<Boolean> isL3) {
    if (isL3 == Optional.of(true)) {
      return DRIVE_KP_KRAKEN;
    } else {
      return DRIVE_KP_NEO;
    }
  }

  /** returns I of PID constants for Drive Motors depending on whether the Module is an L3 or L2 */
  public static final double driveKI(Optional<Boolean> isL3) {
    if (isL3 == Optional.of(true)) {
      return DRIVE_KI_KRAKEN;
    } else {
      return DRIVE_KI_NEO;
    }
  }

  /** returns D of PID constants for Drive Motors depending on whether the Module is an L3 or L2 */
  public static final double driveKD(Optional<Boolean> isL3) {
    if (isL3 == Optional.of(true)) {
      return DRIVE_KD_KRAKEN;
    } else {
      return DRIVE_KD_NEO;
    }
  }

  /**
   * returns S of feedforward constants for Drive Motors depending on whether the Module is an L3 or
   * L2
   */
  public static final double driveKS(Optional<Boolean> isL3) {
    if (isL3 == Optional.of(true)) {
      return DRIVE_KS_KRAKEN;
    } else {
      return DRIVE_KS_NEO;
    }
  }

  /**
   * returns V of feedforward constants for Drive Motors depending on whether the Module is an L3 or
   * L2
   */
  public static final double driveKV(Optional<Boolean> isL3) {
    if (isL3 == Optional.of(true)) {
      return DRIVE_KV_KRAKEN;
    } else {
      return DRIVE_KV_NEO;
    }
  }

  /** Sim Constants */
  /** Moment of inertia of wheel when driving */
  public static final double DRIVE_MOI_KG_M2 = 0.0003125;
  /** Moment of inertia of wheel when turning */
  public static final double STEER_MOI_KG_M2 = 0.0000158025413;

  /** Real Constants */
  /** Wheel Radius in Meters */
  public static final double WHEEL_RADIUS_M = Units.inchesToMeters(2);

  /**
   * Chassis Width, distance between the centerline of two adjacent wheels same for length and width
   * because drivetrain is square
   */
  public static final double TRACK_WIDTH_M = Units.inchesToMeters(22.75);

  /** Max Speed the Robot Can Travel in One Linear Direction (m/s) */
  public static final double MAX_LINEAR_SPEED_M_PER_SEC = 5.2;

  /**
   * Max Speed the Robot Can Rotate (rads/s) Angular Speed is linear speed divided by radius of the
   * "circle" an object moves around (v = wr) The "radius" of the Swerve Drive is equal to half of
   * the distance from one corner to the opposite corner, calculated by sqrt2 * half the track width
   */
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC =
      MAX_LINEAR_SPEED_M_PER_SEC / (Math.sqrt(2) * TRACK_WIDTH_M / 2);

  /** Gear Ratio for MK4I L3 (Kraken) */
  public static final double GEAR_RATIO_L3 = 6.12;
  /** Gear Ratio for MK4I L2 (Neo) */
  public static final double GEAR_RATIO_L2 = 6.75;

  public static final boolean IS_BRAKE_MODE = true;

  /** Used in Robot Characterization Tool to Help Determine Drive Values like PID */
  public static final boolean IS_CHARACTERIZING = false;

  public static final Translation2d[] getModuleTranslations() {
    // Translation 2d assumes that the robot front facing is in the positive x direction and the
    // robot left is in the positive y direction
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0)
    };
  }

  public static enum L2_ABSOLUTE_ENCODER_OFFSET_RAD {
    FRONT_RIGHT(-0.371), // Module 0
    FRONT_LEFT(-2.023), // Module 1
    BACK_LEFT(-0.626), // Module 2
    BACK_RIGHT(0.373); // Module 3

    public final double OFFSET;

    L2_ABSOLUTE_ENCODER_OFFSET_RAD(double value) {
      OFFSET = value;
    }
  }

  public static enum L3_ABSOLUTE_ENCODER_OFFSET_RAD {
    FRONT_RIGHT(-2.169), // Module 0
    FRONT_LEFT(0.8862), // Module 1
    BACK_LEFT(-2.0724), // Module 2
    BACK_RIGHT(0.5599); // Module 3

    public final double OFFSET;

    L3_ABSOLUTE_ENCODER_OFFSET_RAD(double value) {
      OFFSET = value;
    }
  }
  /** Set the inverted for the turn spark max */
  public static final boolean INVERT_TURN_SPARK_MAX = true;
  /** DON'T set the inverted for the drive TalonFX */
  public static final boolean INVERT_DRIVE_TALONFX = false;

  /** Current limiting in amps */
  public static final int CUR_LIM_A = 60;
  /** Enebles the current limit */
  public static final boolean ENABLE_CUR_LIM = true;
  /** Updates encoders every 10 milliseconds */
  public static final int MEASUREMENT_PERIOD_MS = 10;
  /**
   * Within 10% of the desired direction, the joystick is considered to be going in that direction
   */
  public static final double DEADBAND = 0.1;

  /**
   * CAN IDs for motors and encoders /* 1: PDH, 2-5: Absolute Encoders, 6-9: Krakens (Drive), 10-13:
   * Neos (Turn) 14: UTB 15: OTB 16: Actuator 17 & 18: Climbers left and right 19: Indexer
   */
  public static enum ABSOLUTE_ENCODER {
    FRONT_RIGHT(3), // Module 0
    FRONT_LEFT(2), // Module 1
    BACK_LEFT(5), // Module 2
    BACK_RIGHT(4); // Module 3

    public final int ENCODER_ID;

    ABSOLUTE_ENCODER(int ID) {
      ENCODER_ID = ID;
    }
  }

  public enum DRIVE_MOTOR {
    FRONT_RIGHT(7), // Module 0
    FRONT_LEFT(6), // Module 1
    BACK_LEFT(9), // Module 2
    BACK_RIGHT(8); // Module 3

    public final int CAN_ID;

    DRIVE_MOTOR(int value) {
      CAN_ID = value;
    }
  }

  public enum TURN_MOTOR {
    FRONT_RIGHT(11), // Module 0
    FRONT_LEFT(10), // Module 1
    BACK_LEFT(13), // Module 2
    BACK_RIGHT(12); // Module 3

    public final int CAN_ID;

    TURN_MOTOR(int value) {
      CAN_ID = value;
    }
  }

  /** returns gear ratio depending on whether the Module is an L3 or L2 */
  public static final double getGearRatio(boolean isL3) {
    if (isL3) {
      return GEAR_RATIO_L3;
    } else {
      return GEAR_RATIO_L2;
    }
  }
}
