package frc.robot.Subsystems.photonVision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
  /** Offsets the back left camera's position to the center of the robot */
  public static final Transform3d BL_CAMERA_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(-Units.inchesToMeters(10.541), Units.inchesToMeters(11.695), 0),
          new Rotation3d(Math.PI, 0, Math.PI - Units.degreesToRadians(10.881)));

  /** Offsets the back right camera's position to the center of the robot */
  public static final Transform3d BR_CAMERA_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(-Units.inchesToMeters(10.541), -Units.inchesToMeters(11.695), 0),
          new Rotation3d(Math.PI, 0, Math.PI + Units.degreesToRadians(14.881)));

  /** The name of the Lime Light camera */
  public static final String LIME_LIGHT_NAME = "limelight";

  /**
   * The range a NOTE is allowed to be within to stop the robot from rotating during NOTE alignment
   * in autos
   */
  // TODO: Probably best to combine the two but test first
  public static final double LL_AUTONOMOUS_NOTE_RANGE = 5;

  /**
   * The range a NOTE is allowed to be within to stop the robot from rotating during NOTE alignment
   * in teleop
   */
  public static final double LL_TELEOP_NOTE_RANGE = 10;
}
