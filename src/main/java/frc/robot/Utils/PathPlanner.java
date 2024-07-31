// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.DriveConstants;
import frc.robot.Subsystems.gyro.Gyro;

/** Add your docs here. */
public class PathPlanner extends SubsystemBase {
  private Drive drive;
  private PoseEstimator pose;
  private Gyro gyro;

  private boolean speakerRotOverride = false;
  private boolean noteRotOverride = false;

  public PathPlanner(Drive drive, PoseEstimator pose, Gyro gyro) {
    this.drive = drive;
    this.pose = pose;

    AutoBuilder.configureHolonomic(
        pose::getCurrentPose2d,
        pose::resetPose,
        drive::getChassisSpeed,
        drive::runVelocity,
        new HolonomicPathFollowerConfig(
            // new PIDConstants(16, 0, 0.25),
            // new PIDConstants(0.25, 0, 0.025),
            new PIDConstants(1.2, 0, 0.2),
            new PIDConstants(0.3125, 0, 0.025),
            DriveConstants.MAX_LINEAR_SPEED_M_PER_SEC, // Max module speed, in m/s
            DriveConstants.TRACK_WIDTH_M
                / 2, // Drive base radius in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig()),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        drive);

    if (speakerRotOverride) {
      PPHolonomicDriveController.setRotationTargetOverride(pose::AlignToSpeakerPathPlanner);
    }

    if (noteRotOverride) {
      PPHolonomicDriveController.setRotationTargetOverride(drive::noteAlignmentRotationOverride);
    }
  }

  public void enableNOTEAlignment(boolean enable) {
    noteRotOverride = enable;
  }

  public void setSpeakerRotOverrideEnable(boolean enable) {
    speakerRotOverride = enable;
  }

  // public Command followPath() {
  //   return AutoBuilder.followPath(m_path);
  // }
}
