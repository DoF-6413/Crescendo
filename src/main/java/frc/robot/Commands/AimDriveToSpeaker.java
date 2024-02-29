// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.gyro.Gyro;
import frc.robot.Utils.PoseEstimator;

public class AimDriveToSpeaker extends Command {
  /** Creates a new AimDriveToSpeaker. */
  private Drive m_drive;

  private Gyro m_gyro;
  private PoseEstimator m_pose;
  private double x;
  private double y;
  private PIDController rotPID;

  public AimDriveToSpeaker(Drive m_drive, Gyro m_gyro, PoseEstimator m_pose, double x, double y) {
    this.m_drive = m_drive;
    this.m_pose = m_pose;
    this.m_gyro = m_gyro;
    this.x = x;
    this.y = y;
    addRequirements(m_drive, m_gyro, m_pose);
    rotPID = new PIDController(0, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotPID.setSetpoint(speakerAngle());
    // move robot to desired angle

    // if(DriverStation.getAlliance().get() == Alliance.Red) {
    //   allianceOffset = Math.PI;
    // } else {
    //   allianceOffset = 0;
    // }
    m_drive.driveWithDeadband(x, y, rotPID.calculate(m_gyro.getYaw().getDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double speakerAngle() {
    Pose2d dtvalues = m_pose.getCurrentPose2d();
    // 	//triangle for robot angle
    double deltaY = Math.abs(dtvalues.getY() - Field.SPEAKER_Y);
    ;
    double deltaX;
    double m_desiredRobotAngle;
    double speakerDist;
    if (RobotStateConstants.getAlliance().get() == Alliance.Red) {
      deltaX = Math.abs(dtvalues.getX() - Field.RED_SPEAKER_X);
    } else {
      deltaX = Math.abs(dtvalues.getX() - Field.BLUE_SPEAKER_X);
    }
    speakerDist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

    if (dtvalues.getY() >= Field.SPEAKER_Y) {
      // the robot is to the left of the speaker
      double thetaAbove = -Math.toDegrees(Math.asin(deltaX / speakerDist)) - 90;
      m_desiredRobotAngle = thetaAbove;
    } else {
      double thetaBelow = Math.toDegrees(Math.asin(deltaX / speakerDist)) + 90;
      m_desiredRobotAngle = thetaBelow;
    }
    return m_desiredRobotAngle;
  }
}
