// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.drive.Drive;

public class DefaultDriveCommand extends Command {
  /** Creates a new DefaultDriveCommand. */
  CommandXboxController controller;

  Drive drive;
  int index = 0;
  boolean alreadyPressed = false;

  public DefaultDriveCommand(Drive drive, CommandXboxController controller) {
    this.controller = controller;
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (index % 2 == 0) {

      drive.driveWithDeadband(
          controller.getLeftX(), // Forward/backward
          -controller.getLeftY(), // Left/Right (multiply by -1 bc controller axis inverted)
          controller.getRightX()); // Rotate chassis left/right
    } else {

      drive.driveWithDeadbandPlusHeading(
          controller.getLeftX(), // Forward/backward
          -controller.getLeftY(), // Left/Right (multiply by -1 bc controller axis inverted)
          -controller.getRightX()); // Rotate chassis left/right
    }

    if (controller.button(10).getAsBoolean() && alreadyPressed != true) {
      index += 1;
      alreadyPressed = true;
    } else if (!controller.button(10).getAsBoolean() && alreadyPressed == true) {
      alreadyPressed = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.driveWithDeadband(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
