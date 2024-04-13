// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.gyro.Gyro;

/** Add your docs here. */
public class PathPlannerDrive {
  PoseEstimatorLimelight pose;
  Drive drive;
  Gyro gyro;
  HeadingController headingController;

  public PathPlannerDrive(PoseEstimatorLimelight pose, Drive drive, Gyro gyro) {
    this.pose = pose;
    this.drive = drive;
    this.gyro = gyro;
    headingController = new HeadingController();
  }

  public void pathPlannerDrive(ChassisSpeeds chassisSpeeds) {
    drive.runVelocity(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.vxMetersPerSecond,
            headingController.update(
                pose.AngleForSpeaker().plus(new Rotation2d(Math.PI / 2)),
                drive.getRotation(),
                gyro.getRate()),
            drive.getRotation()));
  }
}
