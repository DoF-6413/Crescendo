// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.pose.PoseEstimator;

/** Add your docs here. */
public class PathPlanner extends SubsystemBase {

    private Drive drive;
    private PoseEstimator pose;

    public PathPlanner(Drive drive,PoseEstimator pose){

    AutoBuilder.configureHolonomic(
        pose::getCurrentPose2d,
        pose::resetPose,
        null,
        null,
          new HolonomicPathFollowerConfig(
             new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
             new PIDConstants(0.0, 0.0, 0.0), // Rotation PID constants
             0.0, // Max module speed, in m/s
             DriveConstants.TRACK_WIDTH_M, // Drive base radius in meters. Distance from robot center to furthest module.
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
        drive

    );
}
}
