// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SpeakerScore;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.feeder.FeederConstants;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterConstants;
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
    m_feeder = feeder;
    m_beam = beam;
    m_pose = pose;
    this.controller = controller;
    addRequirements(shooter, wrist, arm, feeder, beam);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feeder.setSetpoint(FeederConstants.REVERSE_RPM);
    m_shooter.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_beam.getShooterSensor() == false) {
      m_feeder.setSetpoint(0);
      m_shooter.setSetpoint(ShooterConstants.MID_RANGE_RPM);
    }

    m_wrist.autoAlignWrist(m_pose.getCurrentPose2d());
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
