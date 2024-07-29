// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.VisionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Utils.BeamBreak;
import frc.robot.Utils.LimelightHelpers;

public class DriveToNote extends Command {
  public final Drive drive;
  public BeamBreak beamBreak;
  double TX;
  double x;
  double y;
  double rot;
  boolean end;
  // public double TY;

  /** Creates a new AlignToNote. */
  public DriveToNote(Drive drive, BeamBreak beamBreak, double x, double y, double rotSpeed) {
    this.drive = drive;
    this.beamBreak = beamBreak;
    this.x = x;
    this.y = y;
    rot = rotSpeed;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
    TX = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TX = LimelightHelpers.getTX("limelight");

    // if (SmartDashboard.getBoolean("Is Note Picked Up", false)) {
    //   x = 0;
    //   y = 0;
    // }

    if (beamBreak.getShooterSensor() == false) {
      end = true;
    }

    if (TX < -5.0) {
      drive.driveWithDeadband(x, y, rot);
    } else if (TX > 5.0) {
      drive.driveWithDeadband(x, y, -rot);
    } else {
      drive.driveWithDeadband(x, y, 0);
    }

    System.out.println("[][][][][][]DRIVING[][][][][][][]");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setRaw(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !beamBreak.getShooterSensor() || end == true;
  }
}
