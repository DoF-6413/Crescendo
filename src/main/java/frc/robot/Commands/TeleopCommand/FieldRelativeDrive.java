// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.DriveConstants;

public class FieldRelativeDrive extends Command {
  private final Drive m_drive;
  private final double m_x;
  private final double m_y;
  private final double m_rot;
  /**
   * The Default Command for the m_drivetrain enableing the m_driver to m_drive
   *
   * @param x velociy in x direction of Entire Swerve m_drive
   * @param y velocity in y direction of Entire Swerve m_drive
   * @param rot Angular Velocity of Entire Swerve m_drive
   */
  public FieldRelativeDrive(double x, double y, double rot, Drive m_drive) {
    this.m_drive = m_drive;
    m_x = x;
    m_y = y;
    m_rot = rot;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Sets to not move on start up
    m_drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, m_drive.getRotation()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(m_x, m_y), DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(m_x, m_y);
    double omega = MathUtil.applyDeadband(m_rot, DriveConstants.DEADBAND);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // The actual run command itself
    m_drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_SEC,
            linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_SEC,
            omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
            m_drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Sets to not move on end
    m_drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, m_drive.getRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
