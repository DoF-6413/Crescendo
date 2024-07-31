// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.VisionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.photonVision.VisionConstants;
import frc.robot.Utils.LimelightHelpers;

public class AlignToNoteDrive extends Command {
  public final Drive drive;
  public CommandXboxController controller;
  public double TX;
  // public double TY;

  /** Creates a new AlignToNote. */
  public AlignToNoteDrive(Drive drive, CommandXboxController controller) {
    this.drive = drive;
    this.controller = controller;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TX = 0.0;
    // TY = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TX = LimelightHelpers.getTX(VisionConstants.LIME_LIGHT_NAME);
    // TY = LimelightHelpers.getTY(VisionConstants.LIME_LIGHT_NAME);

    if (TX < -5.0) {
      drive.driveWithDeadband(controller.getLeftX(), -controller.getLeftY(), 0.3);
    } else if (TX > 5.0) {
      drive.driveWithDeadband(controller.getLeftX(), -controller.getLeftY(), -0.3);
    } else {
      drive.driveWithDeadband(
          controller.getLeftX(), -controller.getLeftY(), -controller.getRightX());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setRaw(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !controller.rightTrigger().getAsBoolean();
  }
}
