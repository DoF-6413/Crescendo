// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.gyro.Gyro;
import frc.robot.Utils.HeadingController;
import frc.robot.Utils.PoseEstimator;
import java.util.function.BooleanSupplier;

public class DefaultDriveCommand extends Command {
  /** Creates a new DefaultDriveCommand. */
  CommandXboxController controller;

  Drive drive;
  Gyro gyro;
  HeadingController headingController;
  PoseEstimator pose;

  int index = 1;
  int prevIndex = index;
  boolean alreadyPressedL3 = false;
  boolean alreadyPressedTrigger = false;
  BooleanSupplier ampSlowdown;
  BooleanSupplier feeding;
  double velocityScaler = 1.0;
  double feedingAngleDeg;

  public DefaultDriveCommand(
      Drive drive,
      Gyro gyro,
      PoseEstimator pose,
      CommandXboxController controller,
      int startingIndex,
      BooleanSupplier feeding,
      BooleanSupplier ampSlowdown) {
    this.controller = controller;

    this.drive = drive;
    this.gyro = gyro;
    this.pose = pose;
    this.ampSlowdown = ampSlowdown;
    this.feeding = feeding;
    index = startingIndex;
    headingController = new HeadingController();
    feedingAngleDeg =
        (RobotStateConstants.getAlliance().isPresent()
                && RobotStateConstants.getAlliance().get() == DriverStation.Alliance.Red
            ? -38 + 180
            : -38);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {

    // If the Arm is over 28 degrees then reduce the max speed of the robot
    if (ampSlowdown.getAsBoolean()) {
      velocityScaler = 0.65;
    } else {
      velocityScaler = 1;
    }

    if (controller.leftTrigger().getAsBoolean() || controller.rightTrigger().getAsBoolean()) {
      /* Auto Rotates Chassis to Align With a NOTE */
      drive.driveWithNoteDetection(
          controller.getLeftX() * velocityScaler, -controller.getLeftY() * velocityScaler, 0.3);

    } else if (feeding.getAsBoolean()
        && !SmartDashboard.getBoolean("HeadingControllerAtSetpoint", false)) {
      drive.driveWithDeadbandForAutoAlign(
          controller.getLeftX() * velocityScaler,
          -controller.getLeftY() * velocityScaler,
          headingController.update(
              Rotation2d.fromDegrees(feedingAngleDeg), pose.getRotation(), gyro.getRate()));
    } else {
      /* Normal Drive Mode */
      drive.driveWithDeadband(
          controller.getLeftX() * velocityScaler, // Forward/backward
          -controller.getLeftY()
              * velocityScaler, // Left/Right (multiply by -1 bc controller a())is inverted)
          -controller.getRightX() * velocityScaler); // Rotate chassis left/right
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
