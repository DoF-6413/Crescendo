// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Utils.PoseEstimator;

public class AimShooter extends Command {
  public Shooter m_Shooter;
  public Wrist m_Wrist;
  public PoseEstimator m_pose;

  /** Creates a new AimShooter. */
  public AimShooter(Shooter shooter, Wrist wrist) {
    addRequirements(shooter, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d dtvalues = m_pose.getCurrentPose2d();
    //triangle for robot angle
    double deltaY = Math.abs(dtvalues.getY() - FieldConstants.SPEAKER_Y);

    double deltaX,speakerDist;

    if (RobotStateConstants.getAlliance().get() == Alliance.Red) {
      deltaX = Math.abs(dtvalues.getX() - FieldConstants.RED_SPEAKER_X);
    } 
    else {
      deltaX = Math.abs(dtvalues.getX() - FieldConstants.BLUE_SPEAKER_X);
    }
    speakerDist = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));

    m_Wrist.setWristSetpoint(m_Shooter.returnDesiredAngle(speakerDist));
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
