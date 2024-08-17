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
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
  private Gyro gyro;

  private SwerveDrivePoseEstimator poseEstimator;
  private PhotonPoseEstimator visionBLPoseEstimator;
  private PhotonPoseEstimator visionBRPoseEstimator;

  private PhotonCamera cameraLeft;
  private PhotonCamera cameraRight;

  public PhotonPipelineResult tempPipelineResult;
  public double resultsTimeStampBL;
  public double resultsTimeStampBR;
  private PhotonTrackedTarget tempTarget;
  private double previousPipelineTimestampBL = 0;
  private double previousPipelineTimestampBR = 0;
  private double poseAmbiguityLeft = 0;
  private double poseAmbiguityRight = 0;
  private int fiducialIDLeft = 0;
  private int fiducialIDRight = 0;
  private boolean hasTargetsLeft = false;
  private boolean hasTargetsRight = false;

  private boolean enable = true;

  /** Used to count how many times the vision part of the periodic has ran. Every 5th run it will run the vision part of the robot pose updating */
  private int counter = 0;

  private final AprilTagFieldLayout aprilTagFieldLayout;
  private Field2d field2d;

  /** Pose Estimation aided by PhotonVision */
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
    aprilTagFieldLayout =
        new AprilTagFieldLayout(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTags(),
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getFieldLength(),
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getFieldWidth());

    cameraLeft = new PhotonCamera(VisionConstants.LEFT_CAMERA_NAME);
    cameraRight = new PhotonCamera(VisionConstants.RIGHT_CAMERA_NAME);

    visionBLPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            cameraLeft,
            VisionConstants.BL_CAMERA_ROBOT_OFFSET);
    visionBRPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            cameraRight,
            VisionConstants.BR_CAMERA_ROBOT_OFFSET);
  }

  @Override
  public void periodic() {
    // When ran on the real robot it would overload the command scheduler, causing input delay from
    // joystick to driving
    field2d.setRobotPose(getCurrentPose2d());
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), drive.getRotation(), drive.getSwerveModulePositions());

    if (enable && counter % 5 == 0) {

      Optional<EstimatedRobotPose> leftPose = getBLVisionEstimation();
      Optional<EstimatedRobotPose> rightPose = getBRVisionEstimation();

      if (cameraLeft.getLatestResult().hasTargets()) {
        tempPipelineResult = cameraLeft.getLatestResult();
        tempTarget = tempPipelineResult.getBestTarget();
        hasTargetsLeft = tempPipelineResult.hasTargets();
        fiducialIDLeft = tempTarget.getFiducialId();
        poseAmbiguityLeft = tempTarget.getPoseAmbiguity();
        resultsTimeStampBL = tempPipelineResult.getTimestampSeconds();
      }

      if (cameraRight.getLatestResult().hasTargets()) {
        tempPipelineResult = cameraRight.getLatestResult();
        tempTarget = tempPipelineResult.getBestTarget();
        hasTargetsRight = tempPipelineResult.hasTargets();
        fiducialIDRight = tempTarget.getFiducialId();
        poseAmbiguityRight = tempTarget.getPoseAmbiguity();
        resultsTimeStampBR = tempPipelineResult.getTimestampSeconds();
      }

      if (!hasTargetsLeft || !hasTargetsRight) {

      } else if (hasTargetsLeft && hasTargetsRight) {
        if (previousPipelineTimestampBL != resultsTimeStampBL
            && previousPipelineTimestampBR != resultsTimeStampBR) {
          previousPipelineTimestampBL = resultsTimeStampBL;
          previousPipelineTimestampBR = resultsTimeStampBR;

          if (poseAmbiguityLeft < 0.2
              && poseAmbiguityLeft > 0.0
              && fiducialIDLeft >= 1
              && fiducialIDLeft <= 16
              && poseAmbiguityRight < 0.2
              && poseAmbiguityRight > 0.0
              && fiducialIDRight >= 1
              && fiducialIDRight <= 16
              && leftPose.isPresent()
              && rightPose.isPresent()) {
            // System.out.println(
            //     "Left Position: " + getBLVisionEstimation().estimatedPose.toPose2d().toString());
            // System.out.println(
            //     "Right Position: " +
            // getBRVisionEstimation().estimatedPose.toPose2d().toString());
            // System.out.println("Average Poition: " + averageVisionPoses().toString());
            poseEstimator.addVisionMeasurement(
                averageVisionPoses(
                    leftPose.get().estimatedPose.toPose2d(),
                    rightPose.get().estimatedPose.toPose2d()),
                resultsTimeStampBL);
          }
        }

      } else if (hasTargetsLeft) {
        if (previousPipelineTimestampBL != resultsTimeStampBL) {
          previousPipelineTimestampBL = resultsTimeStampBL;

          if (poseAmbiguityLeft < 0.2
              && poseAmbiguityLeft > 0.0
              && fiducialIDLeft >= 1
              && fiducialIDLeft <= 16
              && leftPose.isPresent()) {
            poseEstimator.addVisionMeasurement(
                leftPose.get().estimatedPose.toPose2d(), resultsTimeStampBL);
          }
        }

      } else {
        if (previousPipelineTimestampBR != resultsTimeStampBR) {
          previousPipelineTimestampBR = resultsTimeStampBR;

          if (poseAmbiguityRight < 0.2
              && poseAmbiguityRight > 0.0
              && fiducialIDRight >= 1
              && fiducialIDRight <= 16
              && rightPose.isPresent()) {
            poseEstimator.addVisionMeasurement(
                rightPose.get().estimatedPose.toPose2d(), resultsTimeStampBL);
          }
        }
      }
    }
    counter++;
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
   * Toggles the use of Vision/Cameras to update the robot's position
   * 
   * @param enable True = enable, False = disable
   */
  public void enableVision(boolean enable) {
    this.enable = enable;
  }

  /**
   * Returns the estimated pose from the back left camera's pipeline result
   *
   * @param result Back Left PhotonPipelineResult
   */
  public Optional<EstimatedRobotPose> getBLVisionEstimation() {
    return visionBLPoseEstimator.update();
  }

  /**
   * Returns the estimated pose from the back right camera's pipeline result
   *
   * @param result Back Right PhotonPipelineResult
   */
  public Optional<EstimatedRobotPose> getBRVisionEstimation() {
    return visionBRPoseEstimator.update();
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

  /** Returns an optional 2d variation of the angle for Speaker method to be used for PathPlanner's rotation target override */
  public Optional<Rotation2d> AlignToSpeakerPathPlanner() {
    return Optional.of(AngleForSpeaker());
  }

  /**
   * Calculates the average position between the Estimated Pose of the Left and Right Cameras
   * 
   * @param leftEstimatedPose
   * @param rightEstimatedPose
   * @return Pose2d with the averaged position 
   */
  private Pose2d averageVisionPoses(Pose2d leftEstimatedPose, Pose2d rightEstimatedPose) {
    // Breaks down componets of the Estimated Pose from the Left Camera
    double leftX = leftEstimatedPose.getX();
    double leftY = leftEstimatedPose.getY();
    Rotation2d leftRot = leftEstimatedPose.getRotation();
    
    // Breaks down componets of the Estimated Pose from the Right Camera
    double rightX = rightEstimatedPose.getX();
    double rightY = rightEstimatedPose.getY();
    Rotation2d rightRot = rightEstimatedPose.getRotation();

    // Averages rotation
    leftRot.plus(rightRot);
    leftRot.div(2);

    // Averages x and y components and returns the values in a new Pose2d
    return new Pose2d(new Translation2d((leftX + rightX) / 2, (leftY + rightY) / 2), leftRot);
  }
}
