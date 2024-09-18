// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.gyro.Gyro;
import frc.robot.Utils.HeadingController;
import frc.robot.Utils.PoseEstimator;

public class DefaultDriveCommand extends Command {
  /** Creates a new DefaultDriveCommand. */
  CommandXboxController controller;

  Drive drive;
  Gyro gyro;
  int index = 1;
  int prevIndex = index;
  boolean alreadyPressedL3 = false;
  boolean alreadyPressedTrigger = false;
  HeadingController headingController;
  PoseEstimator pose;

  public DefaultDriveCommand(
      Drive drive,
      Gyro gyro,
      PoseEstimator pose,
      CommandXboxController controller,
      int startingIndex) {
    this.controller = controller;

    this.drive = drive;
    this.gyro = gyro;
    this.pose = pose;
    index = startingIndex;
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
    // if (index == 0) {
    //   drive.driveWithDeadbandForAutoAlign(
    //       controller.getLeftX(), // Forward/backward
    //       -controller.getLeftY(), // Left/Right (multiply by -1 bc controller axis inverted)
    //       headingController.update(
    //           pose.angleForSpeaker().plus(new Rotation2d(Math.PI / 2)),
    //           drive.getRotation(),
    //           gyro.getRate())); // Rotate chassis left/right

    // } else if (index % 2 == 0 && index > 0) {
    //   drive.driveWithDeadbandPlusHeading(
    //       controller.getLeftX(), // Forward/backward
    //       -controller.getLeftY(), // Left/Right (multiply by -1 bc controller axis inverted)
    //       -controller.getRightX()); // Rotate chassis left/rightc

    // } else
    if (index == -1) {
      drive.driveWithNoteDetection(controller.getLeftX(), -controller.getLeftY(), 0.3);

    } else if (index > 0) {
      drive.driveWithDeadband(
          controller.getLeftX(), // Forward/backward
          -controller.getLeftY(), // Left/Right (multiply by -1 bc controller a())is inverted)
          -controller.getRightX()); // Rotate chassis left/right
    }

    // if (controller.button(9).getAsBoolean() && alreadyPressedL3 != true) {
    //   index += 1;
    //   alreadyPressedL3 = true;
    // } else if (!controller.button(9).getAsBoolean() && alreadyPressedL3 == true) {
    //   alreadyPressedL3 = false;
    // } else if (controller.button(10).getAsBoolean()) {
    //   index = 0;
    // } else
    if ((controller.leftTrigger().getAsBoolean() || controller.rightTrigger().getAsBoolean())
        && alreadyPressedTrigger != true) {
      prevIndex = index;
      index = -1;
      alreadyPressedTrigger = true;
    } else if (!controller.leftTrigger().getAsBoolean()
        && !controller.rightTrigger().getAsBoolean()
        && alreadyPressedTrigger == true) {
      index = prevIndex;
      alreadyPressedTrigger = false;
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
