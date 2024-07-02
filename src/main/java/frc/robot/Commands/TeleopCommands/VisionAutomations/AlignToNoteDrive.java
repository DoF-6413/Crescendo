// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.VisionAutomations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Utils.LimelightHelpers;

public class AlignToNoteDrive extends Command {
  public final Drive drive;
  public static LimelightHelpers vision;

  double x;
  double y;
  double rot;
  public double TX;
  public double TY;

  /** Creates a new AlignToNote. */
  public AlignToNoteDrive(Drive drive, double x, double y, double rot) {
    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = 0.0;
    y = 0.0;
    rot = 0.0;
    TX = 0.0;
    TY = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TX = LimelightHelpers.getTX("limelight");
    TY = LimelightHelpers.getTY("limelight");

    if (TX < -5.0) {
      drive.setRaw(x, y, 0.3);
    } else if (TX > 5.0) {
      drive.setRaw(x, y, -0.3);
    } else {
      drive.setRaw(x, y, rot);
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
    return false;
  }
}
