// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.gyro.Gyro;
import frc.robot.Utils.HeadingController;
import frc.robot.Utils.PoseEstimatorLimelight;

public class DefaultDriveCommand extends Command {
  /** Creates a new DefaultDriveCommand. */
  CommandXboxController controller;

  Drive drive;
  Gyro gyro;
  int index = 1;
  boolean alreadyPressed = false;
  HeadingController headingController;
  PoseEstimatorLimelight pose;

  public DefaultDriveCommand(
      Drive drive, CommandXboxController controller, Gyro gyro, PoseEstimatorLimelight pose) {
    this.controller = controller;

    this.drive = drive;
    this.gyro = gyro;
    this.pose = pose;
    headingController = new HeadingController();
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (index == 0) {
      drive.driveWithDeadbandForAutoAlign(
          controller.getLeftX(), // Forward/backward
          -controller.getLeftY(), // Left/Right (multiply by -1 bc controller axis inverted)
          headingController.update(
              pose.AngleForSpeaker().plus(new Rotation2d(Math.PI / 2)),
              drive.getRotation(),
              gyro.getRate())); // Rotate chassis left/right
    } else if (index % 2 == 0) {

      drive.driveWithDeadbandPlusHeading(
          controller.getLeftX(), // Forward/backward
          -controller.getLeftY(), // Left/Right (multiply by -1 bc controller axis inverted)
          -controller.getRightX()); // Rotate chassis left/right
    } else {

      drive.driveWithDeadband(
          controller.getLeftX(), // Forward/backward
          -controller.getLeftY(), // Left/Right (multiply by -1 bc controller a())is inverted)
          -controller.getRightX()); // Rotate chassis left/right
    }

    if (controller.button(9).getAsBoolean() && alreadyPressed != true) {
      index += 1;
      alreadyPressed = true;
    } else if (!controller.button(9).getAsBoolean() && alreadyPressed == true) {
      alreadyPressed = false;
    } else if (controller.button(10).getAsBoolean()) {
      index = 0;
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
