// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.RobotStateConstants.Mode;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.gyro.Gyro;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class PoseEstimator extends SubsystemBase {

  public static Vector<N3> StatesStandarDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  public static Vector<N3> visionMesumentsStandarDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  private SwerveDrivePoseEstimator poseEstimator;

  private Field2d field2d;
  private Drive drive;
  private Vision vision;
  private Gyro gyro;
  public PhotonPipelineResult pipelineResult;
  public double resultsTimeStamp;
  public double lastTimeStamp;

  public PoseEstimator(SwerveDriveKinematics kinematics, Drive Drive, Vision Vision, Gyro Gyro) {

    field2d = new Field2d();
    this.drive = Drive;
    this.vision = Vision;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Gyro.getYaw(),
            Drive.getSwerveModulePositions(),
            new Pose2d(new Translation2d(), new Rotation2d()));
  }

  @Override
  public void periodic() {
    field2d.setRobotPose(getCurrentPose2d());
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), gyro.getYaw(), drive.getSwerveModulePositions());

    pipelineResult = vision.getResult();
    resultsTimeStamp = pipelineResult.getTimestampSeconds();

    if (resultsTimeStamp != lastTimeStamp && vision.hasTargets()) {

      lastTimeStamp = resultsTimeStamp;

      var target = pipelineResult.getBestTarget();
      var fiducialID = target.getFiducialId();
      if (target.getPoseAmbiguity() > 0.2
          && fiducialID >= 1
          && fiducialID >= 16) { // 0.2 is considered ambiguos

        AprilTagFieldLayout aprilTagFieldLayout =
            new AprilTagFieldLayout(
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTags(),
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getFieldLength(),
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getFieldWidth());

        Pose3d TagPose = aprilTagFieldLayout.getTagPose(fiducialID).get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = TagPose.transformBy(camToTarget);

        Pose3d visionMeasurement = camPose.transformBy(null); // neet to put offsets
        poseEstimator.addVisionMeasurement(
            visionMeasurement.toPose2d(), Timer.getFPGATimestamp(), visionMesumentsStandarDevs);
      } else {
        System.out.println(
            "best to alternate ratio is less than or equal to 0.2 and no apriltag detected");
      }
    }
  }

  public Pose2d getCurrentPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d currentPose2d) {
    poseEstimator.resetPosition(gyro.getYaw(), drive.getSwerveModulePositions(), currentPose2d);
  }

  public void setPose2d() {
    if (RobotStateConstants.getMode() == Mode.SIM) {
      field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    }
  }

  public Pose2d getInitialPose2d() {
    return null;
  }
}
