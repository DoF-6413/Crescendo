// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.VisionCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Utils.BeamBreak;

public class DriveToNote extends Command {
  private Drive drive;
  private BeamBreak beamBreak;
  private Timer timer;
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

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (beamBreak.getIntakeSensor()) {
    //   timer.reset();
    //   timer.restart();
    //   timer.start();
    // }

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
    return beamBreak.getShooterSensor()
        || timer.hasElapsed(CommandConstants.VISION_PICKUP_TIMEOUT_SEC);
  }
}
