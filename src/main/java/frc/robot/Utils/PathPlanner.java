// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.DriveConstants;

/** Add your docs here. */
public class PathPlanner extends SubsystemBase {
  private Drive drive;
  private PoseEstimator pose;

  public PathPlanner(Drive drive, PoseEstimator pose) {
    this.drive = drive;
    this.pose = pose;

    AutoBuilder.configureHolonomic(
        pose::getCurrentPose2d,
        pose::resetPose,
        drive::getChassisSpeed,
        drive::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants( // Propulsion PID constants
                1.9, 1, 0),
                // DriveConstants.DRIVE_KP_KRAKEN, // 0.6
                // DriveConstants.DRIVE_KI_KRAKEN, // 0.1
                // DriveConstants.DRIVE_KD_KRAKEN), // 0.4
            new PIDConstants( // Steer PID constants
                0.1, 0, 0),
                // 2.225, .7, 0.075),
                // DriveConstants.STEER_KP_NEO, // 6.4
                // DriveConstants.STEER_KI_NEO, // 1.2
                // DriveConstants.STEER_KD_NEO), // .03
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
  }

  // public Command followPath() {
  //   return AutoBuilder.followPath(m_path);
  // }
}
