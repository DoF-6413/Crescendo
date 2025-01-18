// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.DriveConstants;

/** Add your docs here. */
public class PathPlanner extends SubsystemBase {
  private Drive drive;
  private PoseEstimator pose;

  private boolean speakerRotOverride = false;
  private boolean noteRotOverride = false;
  
  private RobotConfig robotConfig;
  private ModuleConfig moduleConfig;

  public PathPlanner(Drive drive, PoseEstimator pose) {
    this.drive = drive;
    this.pose = pose;
    moduleConfig = new ModuleConfig(DriveConstants.WHEEL_RADIUS_M, DriveConstants.MAX_LINEAR_SPEED_M_PER_SEC, 1.0, DCMotor.getKrakenX60(1), DriveConstants.GEAR_RATIO_L3, DriveConstants.CUR_LIM_A, 1); //TODO: get coefficient of friction between wheel and carpet
    robotConfig = new RobotConfig(125, 1, moduleConfig, DriveConstants.TRACK_WIDTH_M); // TODO: Verify MOI and Mass

    AutoBuilder.configure(
        pose::getCurrentPose2d,
        pose::resetPose,
        drive::getChassisSpeed,
        drive::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
                PathPlannerConstants.TRANSLATION_KP, 0, PathPlannerConstants.TRANSLATION_KD),
            new PIDConstants(PathPlannerConstants.ROTATION_KP, 0, PathPlannerConstants.ROTATION_KD)
            ),
          robotConfig,
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

  public void periodic() {}

  /**
   * Toggles the rotation target override for a NOTE during PathPlanner paths
   *
   * @param enable True to enable, False to disable
   */
  public void enableNOTEAlignment(boolean enable) {
    noteRotOverride = enable;
  }

  /**
   * Toggles the rotation target override for the SPEAKER during PathPlanner paths
   *
   * @param enable True to enable, False to disable
   */
  public void enableSpeakerAlignment(boolean enable) {
    speakerRotOverride = enable;
  }

  /**
   * Creates a command that drives the robot to the inputed position
   *
   * @param targetPose Pose2d of where the robot should end up
   */
  public Command pathFindToPose(Pose2d targetPose) {
    // The pose to pathfind to
    // The constraints to use while pathfinding
    // The goal end velocity of the robot when reaching the target pose
    return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
  }
}
