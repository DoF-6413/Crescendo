// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.gyro.*;
// import frc.robot.Subsystems.photonVision.*;

/** This class handels the odometry and locates the robots current position */
public class PoseEstimator extends SubsystemBase {
  /**
   * Increase the numbers to trust the model's state estimate less it is a matrix in form of [x, y,
   * theta] or meters, meters, radians
   */
  public static Vector<N3> stateStandardDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * increase the numbers to trust the vision measurements less also in form [x, y, theta] or
   * meters, meters, radians
   */
  public static Vector<N3> visionMeasurementStandardDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  private SwerveDrivePoseEstimator poseEstimator;
  private Drive drive;
  private Gyro gyro;
  private Field2d field2d;

  public PoseEstimator(Drive drive, Gyro gyro) {

    field2d = new Field2d();
    SmartDashboard.putData(field2d);
    this.drive = drive;
    this.gyro = gyro;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(DriveConstants.getModuleTranslations()),
            gyro.getYaw(),
            drive.getSwerveModulePositions(),
            new Pose2d(new Translation2d(), new Rotation2d()));
  }

  @Override
  public void periodic() {
    // When ran on the real robot it would overload the command scheduler, causing input delay from
    // joystick to driving
    // if (RobotStateConstants.getMode() == Mode.SIM) {
    field2d.setRobotPose(getCurrentPose2d());
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), drive.getRotation(), drive.getSwerveModulePositions());

    LimelightHelpers.PoseEstimate limelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (limelightMeasurement.tagCount >= 2) {
      poseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds,
          visionMeasurementStandardDevs);
    }
    //   }
    // }
    // }
  }

  /**
   * @return the current pose in a Pose2d
   */
  public Pose2d getCurrentPose2d() {
    return poseEstimator.getEstimatedPosition();
  }
  /**
   * Resets the pose
   *
   * @param currentPose2d
   */
  public void resetPose(Pose2d currentPose2d) {
    poseEstimator.resetPosition(gyro.getYaw(), drive.getSwerveModulePositions(), currentPose2d);
  }
  /**
   * @return the rotation in a Rotation2d in degrees
   */
  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }
}
