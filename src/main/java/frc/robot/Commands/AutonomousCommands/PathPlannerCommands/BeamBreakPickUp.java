// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.PathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Utils.BeamBreak;

public class BeamBreakPickUp extends Command {
  private UTBIntake utb;
  private Feeder feeder;
  private Shooter shooter;
  private BeamBreak beamBreak;
  private boolean end;
  /** Creates a new BeamBreakPickUp. */
  public BeamBreakPickUp(UTBIntake utb, Feeder feeder, Shooter shooter, BeamBreak beamBreak) {
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
    utb.setUTBIntakePercentSpeed(-1);
    feeder.setSetpoint(1000);
    shooter.setSetpoint(5000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (beamBreak.getShooterSensor() == false) {}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    utb.setUTBIntakePercentSpeed(0);
    feeder.setSetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !beamBreak.getShooterSensor();
  }
}
