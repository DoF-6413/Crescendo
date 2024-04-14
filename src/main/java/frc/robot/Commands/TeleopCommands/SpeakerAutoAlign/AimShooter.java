// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SpeakerAutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Utils.PoseEstimatorLimelight;

public class AimShooter extends Command {
  public Shooter m_shooter;
  public Wrist m_wrist;
  public Arm m_arm;
  public PoseEstimatorLimelight m_pose;
  public Feeder m_feeder;
  private Timer m_timer;

  /** Creates a new AimShooter. */
  public AimShooter(
      Shooter shooter, Wrist wrist, Arm arm, PoseEstimatorLimelight pose, Feeder feeder) {
    m_shooter = shooter;
    m_wrist = wrist;
    m_arm = arm;
    m_pose = pose;
    m_feeder = feeder;
    m_timer = new Timer();
    addRequirements(shooter, wrist, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feeder.setSetpoint(-1000);
    m_timer.reset();
    m_timer.restart();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_timer.hasElapsed(1.0) && m_shooter.bothAtSetpoint()) {
      m_shooter.setSetpoint(5000);
      m_feeder.setSetpoint(0);
    }

    Pose2d dtvalues = m_pose.getCurrentPose2d();
    // triangle for robot angle
    double deltaX = 0.0;
    if (RobotStateConstants.getAlliance().get() == Alliance.Red) {
      deltaX = Math.abs(dtvalues.getX() - FieldConstants.RED_SPEAKER_X);
    } else if (RobotStateConstants.getAlliance().get() == Alliance.Blue) {
      deltaX = Math.abs(dtvalues.getX() - FieldConstants.BLUE_SPEAKER_X);
    }

    double deltaY = Math.abs(dtvalues.getY() - FieldConstants.SPEAKER_Y);
    if (m_timer.hasElapsed(1.0)) {
      m_shooter.setSetpoint(5000);
    }
    double speakerDist = Math.hypot(deltaX, deltaY);
    m_wrist.setGoal(Units.degreesToRadians(m_shooter.returnDesiredAngle(speakerDist)));
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
