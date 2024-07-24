// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.Intakes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Utils.BeamBreak;

public class IntakeShooterRev extends Command {
  public UTBIntake utb;
  public Feeder feeder;
  public Shooter shooter;
  public BeamBreak beamBreak;

  /** Creates a new IntakeShooterRev. */
  public IntakeShooterRev(UTBIntake utb, Feeder feeder, Shooter shooter, BeamBreak beamBreak) {
    this.utb = utb;
    this.feeder = feeder;
    this.shooter = shooter;
    this.beamBreak = beamBreak;

    addRequirements(utb, feeder, shooter, beamBreak);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new UTBIntakeRun(utb, feeder, true, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
