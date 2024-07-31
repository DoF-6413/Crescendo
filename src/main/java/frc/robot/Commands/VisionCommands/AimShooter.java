// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.VisionCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Utils.BeamBreak;
import frc.robot.Utils.PoseEstimator;

public class AimShooter extends Command {
  public Shooter m_shooter;
  public Wrist m_wrist;
  public Arm m_arm;
  public PoseEstimator m_pose;
  public Feeder m_feeder;
  public CommandXboxController controller;
  public BeamBreak m_beam;
  private Timer m_timer;

  /** Creates a new AimShooter. */
  public AimShooter(
      Shooter shooter,
      Wrist wrist,
      Arm arm,
      PoseEstimator pose,
      Feeder feeder,
      CommandXboxController controller,
      BeamBreak beam) {
    m_shooter = shooter;
    m_wrist = wrist;
    m_arm = arm;
    m_pose = pose;
    m_feeder = feeder;
    this.controller = controller;
    m_beam = beam;
    m_timer = new Timer();
    addRequirements(shooter, wrist, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feeder.setSetpoint(-400);
    m_shooter.setSetpoint(0);
    m_timer.reset();
    m_timer.restart();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_beam.getShooterSensor() == false) {
      m_feeder.setSetpoint(0);
      m_shooter.setSetpoint(5500);
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
    // if (m_timer.hasElapsed(1.0)) {
    //   m_shooter.setSetpoint(5500);
    // }
    double speakerDist = Math.hypot(deltaX, deltaY);
    m_wrist.setGoal(Units.degreesToRadians(m_wrist.returnDesiredAngle(speakerDist)));
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_wrist.setGoal(WristConstants.DEFAULT_POSITION_RAD);
    // m_arm.setGoal(ArmConstants.DEFAULT_POSITION_RAD);
    // m_shooter.setSetpoint(0);
    // m_feeder.setSetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !controller.rightTrigger().getAsBoolean();
  }
}
