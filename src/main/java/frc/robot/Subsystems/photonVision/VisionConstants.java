package frc.robot.Subsystems.photonVision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
  public static final Transform3d cameraBLOnRobotOffsets =
      new Transform3d(
          new Translation3d(-Units.inchesToMeters(10.541), Units.inchesToMeters(11.695), 0),
          new Rotation3d(2 * Math.PI, 0, Math.PI - Units.degreesToRadians(10.881)));

  public static final Transform3d cameraBROnRobotOffsets =
      new Transform3d(
          new Translation3d(-Units.inchesToMeters(10.541), -Units.inchesToMeters(11.695), 0),
          new Rotation3d(2 * Math.PI, 0, Math.PI + Units.degreesToRadians(10.881)));
}
