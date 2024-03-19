// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ZeroCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.climber.Climber;

public class ClimberToZero extends Command {
  /** Creates a new ClimberToZero. */
  private Climber climb;

  public ClimberToZero(Climber climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.setCurrentLimit(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setClimberPercentSpeed(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.setClimberPercentSpeed(0);
    climb.setCurrentLimit(40);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climb.getCurrentDraw() >= 10 ? true : false;
  }
}
