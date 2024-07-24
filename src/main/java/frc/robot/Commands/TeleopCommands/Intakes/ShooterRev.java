// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.Intakes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterConstants;
import frc.robot.Utils.BeamBreak;

public class ShooterRev extends Command {
  public Feeder feeder;
  public Shooter shooter;
  public BeamBreak beamBreak;

  /** Creates a new IntakeShooterRev. */
  public ShooterRev(Feeder feeder, Shooter shooter, BeamBreak beamBreak) {
    this.feeder = feeder;
    this.shooter = shooter;
    this.beamBreak = beamBreak;

    addRequirements(feeder, shooter, beamBreak);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.setSetpoint(-400);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (beamBreak.getShooterSensor() == false) {
      feeder.setSetpoint(0);
      shooter.setSetpoint(ShooterConstants.PRE_REV_RPM);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getSetpoint() > 0;
  }
}
