// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.gyro.Gyro;

public class quickAuto{
  public Drive drive;
  public Timer timer; 
  private double time;
  private double xSpeed;
  private double ySpeed;
  private double RotationSpeed;
  
  /** Creates a new QuickAuto.*/
  public void QuickAuto(Drive drive, double xSpeed, double ySpeed, double rotSpeed, double time ) {
    // Use addRequirements() here to declare subsystem dependencies.
 
    this.drive = drive;
    this.time = time;
    this.xSpeed = xSpeed;
  }
  
  // Called when the command is initially scheduled.

  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
    
    drive.updateHeading();
    // new DefaultDriveCommand(drivetrainSubsystem, gyroSubsystem, ()->0.5, ()-> 0.0, ()-> 0.0).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {
    System.out.println("running");
    // new DefaultDriveCommand(drivetrainSubsystem, gyroSubsystem, ()->0.5, ()-> 0.0, ()-> 0.0);
    drive.setRaw(xSpeed, ySpeed, RotationSpeed);
   
  }

  // Called once the command ends or is interrupted.
  
  public void end(boolean interrupted) {
    // new DefaultDriveCommand(drivetrainSubsystem, gyroSubsystem, ()->0.0, ()-> 0.0, ()-> 0.0);
    System.out.println("end running");
    drive.setRaw(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  
  public boolean isFinished() {
    return timer.get() >= time;
  }
}