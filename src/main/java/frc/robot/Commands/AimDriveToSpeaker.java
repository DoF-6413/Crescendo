// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Utils.PoseEstimator;

public class AimDriveToSpeaker extends Command {
  /** Creates a new AimDriveToSpeaker. */
  private Drive m_drive;

  private PoseEstimator m_pose;
  private double x;
  private double y;
  private CommandXboxController m_xbox;
  private PIDController rotPID;

  public AimDriveToSpeaker(Drive m_drive, PoseEstimator m_pose, CommandXboxController m_xbox) {
    this.m_drive = m_drive;
    this.m_pose = m_pose;
    this.x = m_xbox.getLeftX();
    this.y = m_xbox.getLeftY() * (-1); // Axis inverted
    this.m_xbox = m_xbox;
    addRequirements(m_drive, m_pose);
    rotPID = new PIDController(1.3, 0.1, .001);
    rotPID.enableContinuousInput(-2 * Math.PI, 2 * Math.PI);
    // rotPID.setTolerance(1 / 6 * Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.x = m_xbox.getLeftX();
    this.y = -m_xbox.getLeftY(); // Axis inverted
    rotPID.setSetpoint(speakerAngle());
    // move robot to desired angle
    // if(DriverStation.getAlliance().get() == Alliance.Red) {
    //   allianceOffset = Math.PI;
    // } else {
    //   allianceOffset = 0;
    // }
    m_drive.driveWithDeadband(x, y, -rotPID.calculate(m_drive.getRotation().getRadians()));
    SmartDashboard.putNumber("Current Angle", m_drive.getRotation().getDegrees());
    SmartDashboard.putNumber(
        "Current Calculation", -rotPID.calculate(m_drive.getRotation().getRadians()));
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
    // triangle for robot angle
    double deltaY = Math.abs(dtvalues.getY() - FieldConstants.SPEAKER_Y);
    double deltaX;
    double m_desiredRobotAngle;
    double speakerDist;
    if (RobotStateConstants.getAlliance().get() == Alliance.Red) {
      deltaX = Math.abs(dtvalues.getX() - FieldConstants.RED_SPEAKER_X);
    } else {
      deltaX = Math.abs(dtvalues.getX() - FieldConstants.BLUE_SPEAKER_X);
    }
    speakerDist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)); // pythagoream

    if (dtvalues.getY() >= FieldConstants.SPEAKER_Y) {
      // the robot is to the left of the speaker
      double thetaAbove = -Math.asin(deltaX / speakerDist) - (Math.PI / 2);
      m_desiredRobotAngle = thetaAbove;
    } else {
      double thetaBelow = Math.asin(deltaX / speakerDist) + (Math.PI / 2);
      m_desiredRobotAngle = thetaBelow;
    }
    SmartDashboard.putNumber("Desired Angle in Degrees", (m_desiredRobotAngle));
    return m_desiredRobotAngle;
  }
}