// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommand;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.gyro.Gyro;

public class FieldRelativeDrive extends Command {
  private final Drive m_drive;
  private final double x;
  private final double y;
  private final double rot;
  /** The Default Command for the Drivetrain enableing the driver to drive
   * @param x velociy in x direction of Entire Swerve Drive
   * @param y velocity in y direction of Entire Swerve Drive
   * @param rot Angular Velocity of Entire Swerve Drive
  */
  public FieldRelativeDrive(double x, double y, double rot, Drive m_drive) {
    this.m_drive = m_drive;
    this.x = x;
    this.y = y;
    this.rot = rot;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(x == 0 && y == 0 && rot == 0){
      m_drive.runVelocity(new ChassisSpeeds());
    } else {
      m_drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, m_drive.getRotation()));
    }
  // SwerveModuleState(0, //last position meters)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.runVelocity(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        0, 
        0, 
        0, 
        m_drive.getRotation())
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
