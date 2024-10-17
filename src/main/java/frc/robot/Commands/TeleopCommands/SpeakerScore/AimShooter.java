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
  public Shooter shooter;
  public Wrist wrist;
  public Arm arm;
  public PoseEstimator pose;
  public Feeder feeder;
  public CommandXboxController controller;
  public BeamBreak beamBreak;

  /**
   * Auto Aligns the Wrist to the SPEAKER while also activating the Shooter. This command should
   * only be used in teleop since it ends based on a controller input
   */
  public AimShooter(
      Arm arm,
      Wrist wrist,
      Shooter shooter,
      Feeder feeder,
      PoseEstimator pose,
      BeamBreak beamBreak,
      CommandXboxController controller) {
    this.shooter = shooter;
    this.wrist = wrist;
    this.arm = arm;
    this.feeder = feeder;
    this.beamBreak = beamBreak;
    this.pose = pose;
    this.controller = controller;
    addRequirements(shooter, wrist, arm, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.setSetpoint(FeederConstants.REVERSE_RPM);
    shooter.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (beamBreak.getShooterSensor()) {
      feeder.setSetpoint(0);
      shooter.setSetpoint(ShooterConstants.MID_RANGE_RPM);
    }

    wrist.autoAlignWrist(pose.getCurrentPose2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !controller.rightTrigger().getAsBoolean();
  }
}
