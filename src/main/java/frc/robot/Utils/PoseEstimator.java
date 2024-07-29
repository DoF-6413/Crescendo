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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.gyro.*;
import frc.robot.Subsystems.photonVision.*;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

  private Drive drive;
  private Vision vision;
  private Gyro gyro;

  private SwerveDrivePoseEstimator poseEstimator;
  private PhotonPoseEstimator visionBLPoseEstimator;
  private PhotonPoseEstimator visionBRPoseEstimator;
  public PhotonPipelineResult pipelineResultBL;
  public PhotonPipelineResult pipelineResultBR;
  public double resultsTimeStampBL;
  public double resultsTimeStampBR;
  private double previousPipelineTimestampBL = 0;
  private double previousPipelineTimestampBR = 0;

  private boolean enable = false;

  private final AprilTagFieldLayout aprilTagFieldLayout;
  private Field2d field2d;

  /** Pose Estimation aided by PhotonVision */
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

    visionBLPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            VisionConstants.cameraBLOnRobotOffsets);
    visionBRPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            VisionConstants.cameraBROnRobotOffsets);
  }

  @Override
  public void periodic() {
    // When ran on the real robot it would overload the command scheduler, causing input delay from
    // joystick to driving
    field2d.setRobotPose(getCurrentPose2d());
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), drive.getRotation(), drive.getSwerveModulePositions());

    if (enable) {
      // Adds pose estimated from Back Left camera to Swerve Pose Estimator
      if (vision.getResultBL().hasTargets() == true && vision.getResultBL() != null) {
        pipelineResultBL = vision.getResultBL();
        resultsTimeStampBL = pipelineResultBL.getTimestampSeconds();

        if (resultsTimeStampBL != previousPipelineTimestampBL && pipelineResultBL != null) {
          previousPipelineTimestampBL = resultsTimeStampBL;

          if (pipelineResultBL.getBestTarget() != null
              && pipelineResultBL.getBestTarget().getPoseAmbiguity() < 0.2
              && pipelineResultBL.getBestTarget().getFiducialId() >= 1
              && pipelineResultBL.getBestTarget().getFiducialId() <= 16) {
            poseEstimator.addVisionMeasurement(
                getBLVisionEstimation(pipelineResultBL).estimatedPose.toPose2d(),
                resultsTimeStampBL,
                visionMeasurementStandardDevs);
          }
        }
      }
      // Adds pose estimated from Back Right camera to Swerve Pose Estimator
      if (vision.getResultBR().hasTargets() && vision.getResultBR() != null) {
        pipelineResultBR = vision.getResultBR();
        resultsTimeStampBR = pipelineResultBR.getTimestampSeconds();

        if (resultsTimeStampBR != previousPipelineTimestampBR && pipelineResultBR != null) {
          previousPipelineTimestampBR = resultsTimeStampBR;

          if (pipelineResultBR.getBestTarget() != null
              && pipelineResultBR.getBestTarget().getPoseAmbiguity() < 0.2
              && vision.getResultBR() != null
              && pipelineResultBR.getBestTarget().getFiducialId() >= 1
              && pipelineResultBR.getBestTarget().getFiducialId() <= 16) {
            poseEstimator.addVisionMeasurement(
                getBRVisionEstimation(pipelineResultBR).estimatedPose.toPose2d(),
                resultsTimeStampBR,
                visionMeasurementStandardDevs);
          }
        }
      }
    }

    // if (vision.getResultBL().hasTargets()) {

    //   pipelineResultBL = vision.getResultBL();
    //   resultsTimeStampBL = pipelineResultBL.getTimestampSeconds();

    //   if (resultsTimeStampBL != previousPipelineTimestampBL) {

    //     previousPipelineTimestampBL = resultsTimeStampBL;

    //     var target = pipelineResultBL.getBestTarget();
    //     var fiducialID = target.getFiducialId();
    //     if (target.getPoseAmbiguity() < 0.2
    //         && fiducialID >= 1
    //         && fiducialID <= 16) { // 0.2 is considered ambiguous

    //       Pose3d tagPose = aprilTagFieldLayout.getTagPose(fiducialID).get();
    //       Transform3d camToTarget = target.getBestCameraToTarget();
    //       Pose3d camPose = tagPose.transformBy(camToTarget);

    //       Pose3d visionMeasurement = camPose.transformBy(VisionConstants.cameraBLOnRobotOffsets);
    //       poseEstimator.addVisionMeasurement(
    //           visionMeasurement.toPose2d(),
    //           Timer.getFPGATimestamp(),
    //           visionMeasurementStandardDevs);
    //     }
    //   }
    // }

    // if (vision.getResultBR().hasTargets()) {

    //   pipelineResultBR = vision.getResultBR();
    //   resultsTimeStampBR = pipelineResultBR.getTimestampSeconds();

    //   if (resultsTimeStampBR != previousPipelineTimestampBR) {

    //     previousPipelineTimestampBR = resultsTimeStampBR;

    //     var target = pipelineResultBR.getBestTarget();
    //     var fiducialID = target.getFiducialId();
    //     if (target.getPoseAmbiguity() < 0.2
    //         && fiducialID >= 1
    //         && fiducialID <= 16) { // 0.2 is considered ambiguous

    //       Pose3d tagPose = aprilTagFieldLayout.getTagPose(fiducialID).get();
    //       Transform3d camToTarget = target.getBestCameraToTarget();
    //       Pose3d camPose = tagPose.transformBy(camToTarget);

    //       Pose3d visionMeasurement = camPose.transformBy(VisionConstants.cameraBROnRobotOffsets);
    //       poseEstimator.addVisionMeasurement(
    //           visionMeasurement.toPose2d(),
    //           Timer.getFPGATimestamp(),
    //           visionMeasurementStandardDevs);
    //     }
    //   }
    // }
  }

  /**
   * @return The current pose in a Pose2d
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
    poseEstimator.resetPosition(gyro.getAngle(), drive.getSwerveModulePositions(), currentPose2d);
  }

  /**
   * @return the rotation in a Rotation2d in degrees
   */
  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Returns the estimated pose from the back left camera's pipeline result
   *
   * @param result Back Left PhotonPipelineResult
   */
  public EstimatedRobotPose getBLVisionEstimation(PhotonPipelineResult result) {
    return visionBLPoseEstimator.update(result).get();
  }

  public void enableVision(boolean enable) {
    this.enable = enable;
  }

  /**
   * Returns the estimated pose from the back right camera's pipeline result
   *
   * @param result Back Right PhotonPipelineResult
   */
  public EstimatedRobotPose getBRVisionEstimation(PhotonPipelineResult result) {
    return visionBRPoseEstimator.update(result).get();
  }

  public Rotation2d AngleForSpeaker() {
    Translation2d delta;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      delta = this.getCurrentPose2d().getTranslation().minus(FieldConstants.RED_SPEAKER);
      delta =
          delta.minus(
              new Translation2d(
                      0, (drive.getChassisSpeed().vyMetersPerSecond / 6) * delta.getNorm())
                  .rotateBy(
                      this.getCurrentPose2d().getRotation().plus(Rotation2d.fromDegrees(180))));
      return Rotation2d.fromRadians(Math.atan(delta.getY() / delta.getX()))
          .rotateBy(new Rotation2d(Math.PI));
    } else {
      delta = this.getCurrentPose2d().getTranslation().minus(FieldConstants.BLUE_SPEAKER);
      delta =
          delta.plus(
              new Translation2d(
                      0, (drive.getChassisSpeed().vyMetersPerSecond / 6) * delta.getNorm())
                  .rotateBy(this.getCurrentPose2d().getRotation()));
      return Rotation2d.fromRadians(Math.atan(delta.getY() / delta.getX()));
    }
  }

  public Optional<Rotation2d> AlignToSpeakerPathPlanner() {
    return Optional.of(AngleForSpeaker());
  }
}
