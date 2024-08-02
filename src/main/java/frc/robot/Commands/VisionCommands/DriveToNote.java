// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.VisionCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.photonVision.VisionConstants;
import frc.robot.Utils.BeamBreak;
import frc.robot.Utils.LimelightHelpers;

public class DriveToNote extends Command {
  private Drive drive;
  private BeamBreak beamBreak;
  private Timer timer;
  double TX;
  double x;
  double y;
  double rotSpeed;

  /** Creates a new DriveToNote. */
  public DriveToNote(Drive drive, BeamBreak beamBreak, double x, double y, double rotSpeed) {
    this.drive = drive;
    this.beamBreak = beamBreak;
    this.x = x;
    this.y = y;
    this.rotSpeed = rotSpeed;

    addRequirements(drive, beamBreak);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TX = 0.0;
    timer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TX = LimelightHelpers.getTX(VisionConstants.LIME_LIGHT_NAME);

    if (SmartDashboard.getBoolean("Is Note Picked Up", false) == true) {
      timer.reset();
      timer.start();
    }

    drive.driveWithNoteDetection(x, y, rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setRaw(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !beamBreak.getShooterSensor()
        || timer.hasElapsed(CommandConstants.VISION_PICKUP_TIMEOUT_SEC);
  }
}
