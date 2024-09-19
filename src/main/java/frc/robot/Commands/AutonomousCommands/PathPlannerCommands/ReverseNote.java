// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.PathPlannerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.feeder.FeederConstants;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterConstants;
import frc.robot.Utils.BeamBreak;

public class ReverseNote extends Command {
  private Feeder feeder;
  private Shooter shooter;
  private BeamBreak beamBreak;
  private Timer timer;
  /** Creates a new ReverseFeeder. */
  public ReverseNote(Shooter shooter, Feeder feeder, BeamBreak beamBreak) {
    this.feeder = feeder;
    this.shooter = shooter;
    this.beamBreak = beamBreak;

    addRequirements(feeder, shooter, beamBreak);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.setSetpoint(FeederConstants.REVERSE_RPM);
    shooter.setSetpoint(ShooterConstants.REVERSE_RPM);
    timer = new Timer();
    timer.reset();
    timer.restart();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setSetpoint(0);
    shooter.setSetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !beamBreak.getShooterSensor()
        || timer.hasElapsed(CommandConstants.FEEDER_REVERSE_TIMEOUT_SEC);
  }
}
