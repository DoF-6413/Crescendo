// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SpeakerScore;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Subsystems.wrist.WristConstants;
import frc.robot.Utils.PoseEstimator;

public class AimWrist extends Command {
  public Wrist wrist;
  public Arm arm;
  public PoseEstimator pose;
  // private Timer m_timer;

  /** Updates the angle of the Wrist based on the robot's distance from the SPEAKER */
  public AimWrist(Arm arm, Wrist wrist, PoseEstimator pose) {
    this.wrist = wrist;
    this.arm = arm;
    this.pose = pose;
    // m_timer = new Timer();
    addRequirements(wrist, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_timer.reset();
    // m_timer.restart();
    // m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.autoAlignWrist(pose.getCurrentPose2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.atGoal() && wrist.getGoal() != WristConstants.DEFAULT_POSITION_RAD;
  }
}
