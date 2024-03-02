// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Utils.PoseEstimator;

public class AimShooter extends Command {
  public Shooter m_shooter;
  public Wrist m_wrist;
  public PoseEstimator m_pose;
  double deltaX, deltaY, speakerDist;

  /** Creates a new AimShooter. */
  public AimShooter(Shooter shooter, Wrist wrist, PoseEstimator pose) {
    m_shooter = shooter;
    m_wrist = wrist;
    m_pose = pose;
    addRequirements(shooter, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d dtvalues = m_pose.getCurrentPose2d();
    // triangle for robot angle

    if (RobotStateConstants.getAlliance().get() == Alliance.Red) {
      deltaX = Math.abs(dtvalues.getX() - FieldConstants.RED_SPEAKER_X);
    } else if (RobotStateConstants.getAlliance().get() == Alliance.Blue) {
      deltaX = Math.abs(dtvalues.getX() - FieldConstants.BLUE_SPEAKER_X);
    }

    deltaY = Math.abs(dtvalues.getY() - FieldConstants.SPEAKER_Y);
    speakerDist = Math.hypot(deltaX, deltaY);

    m_wrist.setWristSetpoint(m_shooter.returnDesiredAngle(speakerDist));
    SmartDashboard.putString("abidhaihdaihdia", "ndwaindinidawniaidn");
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}