// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.*;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.RobotStateConstants.Mode;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.gyro.*;
import frc.robot.Subsystems.vision.*;
import org.photonvision.targeting.PhotonPipelineResult;

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
  private Vision vision;
  private Gyro gyro;
  private Field2d field2d;
  public PhotonPipelineResult pipelineResult;
  public double resultsTimeStamp;

  private double previousPipelineTimestamp = 0;
  private final AprilTagFieldLayout aprilTagFieldLayout;

  public PoseEstimator(Drive drive, Gyro gyro, Vision Vision) {

    field2d = new Field2d();
    SmartDashboard.putData(field2d);
    this.drive = drive;
    this.vision = Vision;
    this.gyro = gyro;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(DriveConstants.getModuleTranslations()),
            gyro.getYaw(),
            drive.getSwerveModulePositions(),
            new Pose2d(new Translation2d(), new Rotation2d()));
    aprilTagFieldLayout =
        new AprilTagFieldLayout(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTags(),
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getFieldLength(),
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getFieldWidth());
  }

  @Override
  public void periodic() {
    // When ran on the real robot it would overload the command scheduler, causing input delay from
    // joystick to driving
    if (RobotStateConstants.getMode() == Mode.SIM) {
      field2d.setRobotPose(getCurrentPose2d());
      poseEstimator.updateWithTime(
          Timer.getFPGATimestamp(), drive.getRotation(), drive.getSwerveModulePositions());
    }

    if (vision.getResult().hasTargets()) {

      pipelineResult = vision.getResult();
      resultsTimeStamp = pipelineResult.getTimestampSeconds();

      if (resultsTimeStamp != previousPipelineTimestamp) {

        previousPipelineTimestamp = resultsTimeStamp;

        var target = pipelineResult.getBestTarget();
        var fiducialID = target.getFiducialId();
        if (target.getPoseAmbiguity() < 0.2
            && fiducialID >= 1
            && fiducialID <= 16) { // 0.2 is considered ambiguous

          Pose3d tagPose = aprilTagFieldLayout.getTagPose(fiducialID).get();
          Transform3d camToTarget = target.getBestCameraToTarget();
          Pose3d camPose = tagPose.transformBy(camToTarget);

          Pose3d visionMeasurement = camPose.transformBy(VisionConstants.cameraOnRobotOffsets);
          poseEstimator.addVisionMeasurement(
              visionMeasurement.toPose2d(),
              Timer.getFPGATimestamp(),
              visionMeasurementStandardDevs);
        }
      }
    }
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
