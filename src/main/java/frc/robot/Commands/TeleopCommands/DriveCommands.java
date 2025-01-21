package frc.robot.Commands.TeleopCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.DriveConstants;

public class DriveCommands {

  private DriveCommands() {}

  /** 
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command fieldRelativeDrive(
      Drive drive,
      DoubleSupplier joystickLeftX,
      DoubleSupplier joystickLeftY,
      DoubleSupplier joystickRightX) {
    return Commands.run(
        () -> {
          // Get the Linear Velocity & Omega from inputs
          Translation2d linearVelocity =
              getLinearVelocity(joystickLeftX.getAsDouble(), joystickLeftY.getAsDouble());
          double omega = getOmega(joystickRightX.getAsDouble());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_SEC,
                  -linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_SEC,
                  -omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC);
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Compute the new linear velocity from inputs, including applying deadbands and squaring for
   * smoothness. Also apply the linear velocity Slew Rate Limiter.
   */
  private static Translation2d getLinearVelocity(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    // NOTE: The x & y values range from -1 to +1, so their squares are as well
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Compute the new angular velocity from inputs, including applying deadbands and squaring for
   * smoothness. Also apply the angular Slew Rate Limiter.
   */
  private static double getOmega(double omega) {
    omega = MathUtil.applyDeadband(omega, DriveConstants.DEADBAND);
    return Math.copySign(omega * omega, omega);
  } 
}
