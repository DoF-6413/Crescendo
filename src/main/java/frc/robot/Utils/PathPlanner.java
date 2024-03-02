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

  public PathPlanner(Drive drive, PoseEstimator pose) {

    AutoBuilder.configureHolonomic(
        pose::getCurrentPose2d,
        pose::resetPose,
        drive::getChassisSpeed,
        drive::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants( // Propulsion PID constants
                DriveConstants.DRIVE_KP_KRAKEN,
                DriveConstants.DRIVE_KI_KRAKEN,
                DriveConstants.DRIVE_KD_KRAKEN),
            new PIDConstants( // Steer PID constants
            1,0,0),
                // DriveConstants.STEER_KP_NEO,
                // DriveConstants.STEER_KI_NEO,
                // DriveConstants.STEER_KD_NEO),
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
