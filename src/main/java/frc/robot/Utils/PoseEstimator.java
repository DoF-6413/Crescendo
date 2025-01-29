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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.gyro.*;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
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
  public static final Vector<N3> stateStandardDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * increase the numbers to trust the vision measurements less also in form [x, y, theta] or
   * meters, meters, radians
   */
  public static final Vector<N3> visionMeasurementStandardDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  private final Drive drive;
  private final Gyro gyro;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final PhotonPoseEstimator visionPoseEstimatorLeft;
  private final PhotonPoseEstimator visionPoseEstimatorRight;

  private final PhotonCamera cameraLeft;
  private final PhotonCamera cameraRight;

  private double timestamp;
  private double prevTimestamp = 0;

  private PhotonPipelineResult tempPipelineResult;
  private PhotonTrackedTarget leftTarget;
  private PhotonTrackedTarget rightTarget;
  private double poseAmbiguityLeft = 0;
  private double poseAmbiguityRight = 0;
  private int fiducialIDLeft = 0;
  private int fiducialIDRight = 0;
  private boolean hasTargetsLeft = false;
  private boolean hasTargetsRight = false;

  private boolean enable = true;

  /**
   * Used to count how many times the vision part of the periodic has ran. Every 5th run it will run
   * the vision part of the robot pose updating
   */
  // private int counter = 0;

  /**
   * The amount of cycle of the periodic that need to run before Vision is allowed to update the
   * Pose Estimator
   */
  // private int cyclesPerUpdate = 5;

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
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags(),
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getFieldLength(),
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getFieldWidth());

    cameraLeft = new PhotonCamera(VisionConstants.LEFT_CAMERA_NAME);
    cameraRight = new PhotonCamera(VisionConstants.RIGHT_CAMERA_NAME);

    visionPoseEstimatorLeft = 
      new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.LOWEST_AMBIGUITY, 
            VisionConstants.LEFT_CAMERA_ROBOT_OFFSET);
    visionPoseEstimatorRight =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            VisionConstants.RIGHT_CAMERA_ROBOT_OFFSET);
  }

  @Override
  public void periodic() {
    timestamp = Timer.getFPGATimestamp();

    // When ran on the real robot it would overload the command scheduler, causing input delay from
    // joystick to driving
    field2d.setRobotPose(getCurrentPose2d());
    poseEstimator.updateWithTime(timestamp, drive.getRotation(), drive.getSwerveModulePositions());

    // counter++;
    // if (enable && counter % cyclesPerUpdate == 0 && RobotStateConstants.getMode() ==
    // RobotStateConstants.Mode.REAL) {
    if (true) {
      hasTargetsLeft = false;
      hasTargetsRight = false;
      
      // Saves pipeline results from left camera if present
      tempPipelineResult = cameraLeft.getLatestResult();
      Optional<EstimatedRobotPose> leftPose = visionPoseEstimatorLeft.update(tempPipelineResult);
      if (tempPipelineResult.hasTargets()) {
        hasTargetsLeft = tempPipelineResult.hasTargets();
        leftTarget = tempPipelineResult.getBestTarget();
        fiducialIDLeft = leftTarget.getFiducialId();
        poseAmbiguityLeft = leftTarget.getPoseAmbiguity();
      }
      
      // Saves pipeline results from right camera if present
      tempPipelineResult = cameraRight.getLatestResult();
      Optional<EstimatedRobotPose> rightPose = visionPoseEstimatorRight.update(tempPipelineResult);
      if (tempPipelineResult.hasTargets()) {
        hasTargetsRight = tempPipelineResult.hasTargets();
        rightTarget = tempPipelineResult.getBestTarget();
        fiducialIDRight = rightTarget.getFiducialId();
        poseAmbiguityRight = rightTarget.getPoseAmbiguity();
      }

      Logger.recordOutput("Vision/Back_Left/HasTarget", hasTargetsLeft);;
      Logger.recordOutput("Vision/Back_Right/HasTarget", hasTargetsRight);;

      if (!hasTargetsLeft && !hasTargetsRight) {
        return;

      } else if (hasTargetsLeft && hasTargetsRight) {
        if (prevTimestamp != timestamp) {
          prevTimestamp = timestamp;

          if (leftPose.isPresent()
              && rightPose.isPresent()
              && poseAmbiguityLeft < 0.2
              && poseAmbiguityLeft > 0.0
              && poseAmbiguityRight < 0.2
              && poseAmbiguityRight > 0.0
              && fiducialIDLeft >= 1
              && fiducialIDLeft <= 22
              && fiducialIDRight >= 1
              && fiducialIDRight <= 22) {
            Logger.recordOutput("Vision/Back_Left/EstimatedPose", leftPose.get().estimatedPose.toPose2d());
            Logger.recordOutput("Vision/Back_right/EstimatedPose", rightPose.get().estimatedPose.toPose2d());
            poseEstimator.addVisionMeasurement(
                averageVisionPoses(
                    leftPose.get().estimatedPose.toPose2d(),
                    rightPose.get().estimatedPose.toPose2d()),
                timestamp);
          }
        }

      } else if (hasTargetsLeft) {
        if (prevTimestamp != timestamp) {
          prevTimestamp = timestamp;

          if (leftPose.isPresent()
              && poseAmbiguityLeft < 0.2
              && poseAmbiguityLeft > 0.0
              && fiducialIDLeft >= 1
              && fiducialIDLeft <= 22) {
            Logger.recordOutput("Vision/Back_Left/EstimatedPose", leftPose.get().estimatedPose.toPose2d());
            poseEstimator.addVisionMeasurement(leftPose.get().estimatedPose.toPose2d(), timestamp);
          }
        }

      } else {
        if (prevTimestamp != timestamp) {
          prevTimestamp = timestamp;

          if (rightPose.isPresent()
              && poseAmbiguityRight < 0.2
              && poseAmbiguityRight > 0.0
              && fiducialIDRight >= 1
              && fiducialIDRight <= 22) {
            Logger.recordOutput("Vision/Back_right/EstimatedPose", rightPose.get().estimatedPose.toPose2d());
            poseEstimator.addVisionMeasurement(rightPose.get().estimatedPose.toPose2d(), timestamp);
          }
        }
      }
    }
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
   * @param currentPose2d Position to set the robot to
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

  /**
   * Toggles the use of Vision/Cameras to update the robot's position
   *
   * @param enable True = enable, False = disable
   */
  public void enableVision(boolean enable) {
    this.enable = enable;
  }

  public Rotation2d angleForSpeaker() {
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

  /**
   * Returns an optional 2d variation of the angle for Speaker method to be used for PathPlanner's
   * rotation target override
   */
  public Optional<Rotation2d> alignToSpeakerPathPlanner() {
    return Optional.of(angleForSpeaker());
  }

  /**
   * Calculates the average position between the Estimated Poses from the Vision
   *
   * @param estimatedPoses Poses to average
   * @return Pose2d with the averaged position
   */
  private Pose2d averageVisionPoses(Pose2d... estimatedPoses) {
    double x = 0;
    double y = 0;
    double theta = 0;
    for (Pose2d pose : estimatedPoses) {
      x += pose.getX();
      y += pose.getY();
      theta += pose.getRotation().getRadians();
    }

    // Averages x, y and theta components and returns the values in a new Pose2d
    return new Pose2d(
        new Translation2d(x / estimatedPoses.length, y / estimatedPoses.length),
        new Rotation2d(theta / estimatedPoses.length));
  }

  // public Pose2d toAprilTag() {
  //   Pose2d robotToInFrontOfTarget = new Pose2d();
  //   if (!hasTargetsLeft && !hasTargetsRight) {
  //     return this.getCurrentPose2d();
  //   }

  //   if (hasTargetsLeft && hasTargetsRight) {

  //     var leftCam = leftTarget.bestCameraToTarget.plus(VisionConstants.LEFT_CAMERA_ROBOT_OFFSET);
  //     var rightCam = leftTarget.bestCameraToTarget.plus(VisionConstants.RIGHT_CAMERA_ROBOT_OFFSET); 
  //     var tagPose2d = aprilTagFieldLayout.getTagPose(leftTarget.getFiducialId()).get().toPose2d();
  //     var inFrontOfTag = 
  //       tagPose2d.plus(new Transform2d(
  //           tagPose2d.getX() + Units.inchesToMeters(5) * tagPose2d.getRotation().getCos(),
  //           tagPose2d.getY() + Units.inchesToMeters(5) * tagPose2d.getRotation().getSin(),
  //           tagPose2d.getRotation()
  //           ));
  //     robotToInFrontOfTarget = this.averageVisionPoses(
  //     new Pose2d(leftCam.getTranslation().getX() + inFrontOfTag.getX(), leftCam.getTranslation().getY() + inFrontOfTag.getY(), Rotation2d.fromRadians(leftTarget.bestCameraToTarget.getRotation().getZ())), 
  //     new Pose2d(rightCam.getTranslation().getX() + inFrontOfTag.getX(), rightCam.getTranslation().getY() + inFrontOfTag.getY(), Rotation2d.fromRadians(rightTarget.bestCameraToTarget.getRotation().getAngle())));

  //   } else if (hasTargetsLeft) {
  //     var leftCam = leftTarget.bestCameraToTarget.plus(VisionConstants.LEFT_CAMERA_ROBOT_OFFSET);
  //     var tagPose2d = aprilTagFieldLayout.getTagPose(leftTarget.getFiducialId()).get().toPose2d();
  //     var inFrontOfTag = 
  //     tagPose2d.plus(new Transform2d(
  //       tagPose2d.getX() + Units.inchesToMeters(5) * tagPose2d.getRotation().getCos(),
  //       tagPose2d.getY() + Units.inchesToMeters(5) * tagPose2d.getRotation().getSin(),
  //       tagPose2d.getRotation()
  //       ));
  //       robotToInFrontOfTarget = new Pose2d(leftCam.getTranslation().getX() + inFrontOfTag.getX(), leftCam.getTranslation().getY() + inFrontOfTag.getY(), Rotation2d.fromRadians(leftTarget.bestCameraToTarget.getRotation().getZ()));
        
  //     } else {
  //       var rightCam = leftTarget.bestCameraToTarget.plus(VisionConstants.RIGHT_CAMERA_ROBOT_OFFSET); 
  //       var tagPose2d = aprilTagFieldLayout.getTagPose(leftTarget.getFiducialId()).get().toPose2d();
  //       var inFrontOfTag = 
  //       tagPose2d.plus(new Transform2d(
  //           tagPose2d.getX() + Units.inchesToMeters(5) * tagPose2d.getRotation().getCos(),
  //           tagPose2d.getY() + Units.inchesToMeters(5) * tagPose2d.getRotation().getSin(),
  //           tagPose2d.getRotation()
  //           ));
  //       robotToInFrontOfTarget = new Pose2d(rightCam.getTranslation().getX() + inFrontOfTag.getX(), rightCam.getTranslation().getY() + inFrontOfTag.getY(), Rotation2d.fromRadians(rightTarget.bestCameraToTarget.getRotation().getAngle()));
  //     }
  //     return this.getCurrentPose2d().plus(new Transform2d(robotToInFrontOfTarget.getTranslation(), robotToInFrontOfTarget.getRotation()));
  // }

  public Pose2d toAprilTag() {
    System.out.println("-==-=-=-==DOING THE THINGY-=-=-=-=-=-=-");
    System.out.println("-==-=-=-==DOING THE THINGY-=-=-=-=-=-=-");
    System.out.println("-==-=-=-==DOING THE THINGY-=-=-=-=-=-=-");
    System.out.println("-==-=-=-==DOING THE THINGY-=-=-=-=-=-=-");
    System.out.println("-==-=-=-==DOING THE THINGY-=-=-=-=-=-=-");
    System.out.println("-==-=-=-==DOING THE THINGY-=-=-=-=-=-=-");
    if (!hasTargetsLeft && !hasTargetsRight) {
      return this.getCurrentPose2d();
    }

    // if (hasTargetsLeft && hasTargetsRight) {
    //   if (leftTarget.getFiducialId()==rightTarget.getFiducialId()){

    //   }
    // } // TODO: Figure out how to decide which tag to use if different tags are seen by cameras

    if (hasTargetsLeft) {
      var tagPose2d = aprilTagFieldLayout.getTagPose(leftTarget.getFiducialId()).get().toPose2d();
      var inFrontOfTag = new Pose2d(tagPose2d.getX() + Units.inchesToMeters(5)*tagPose2d.getRotation().getCos() , tagPose2d.getY() + Units.inchesToMeters(5)*tagPose2d.getRotation().getSin(), tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180)));
      return inFrontOfTag;
    } else {
      var tagPose2d = aprilTagFieldLayout.getTagPose(rightTarget.getFiducialId()).get().toPose2d();
      var inFrontOfTag = new Pose2d(tagPose2d.getX() + Units.inchesToMeters(5)*tagPose2d.getRotation().getCos() , tagPose2d.getY() + Units.inchesToMeters(5)*tagPose2d.getRotation().getSin(), tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180)));
      return inFrontOfTag;
    }
  }
}
