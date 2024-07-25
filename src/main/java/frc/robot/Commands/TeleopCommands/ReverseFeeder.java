// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Utils.BeamBreak;

public class ReverseFeeder extends Command {
  private Feeder feeder;
  private BeamBreak beamBreak;
  /** Creates a new ReverseFeeder. */
  public ReverseFeeder(Feeder feeder, BeamBreak beamBreak) {
    this.feeder = feeder;
    this.beamBreak = beamBreak;
    addRequirements(feeder, beamBreak);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.setSetpoint(-500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setSetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !beamBreak.getShooterSensor();
  }
}
