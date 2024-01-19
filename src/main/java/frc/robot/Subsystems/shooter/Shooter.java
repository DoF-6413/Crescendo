// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter extends PIDSubsystem{
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public ProfiledPIDController shooterProfiledPIDController;

  public Shooter(ShooterIO io) {
    shooterProfiledPIDController = 
    new ProfiledPIDController(
      ShooterConstants.kP,
      ShooterConstants.kI,
      ShooterConstants.kD,
        new TrapezoidProfile.Constraints(
          ShooterConstants.MAX_VELOCITY,
          ShooterConstants.MAX_ACCELERATION));
          
      System.out.println("[Init] Creating Shooter");
      this.io = io;
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void setShooterMotorsVoltage(double volts) {
    io.setShooterMotorsVoltage(volts);
  }

  public void setShooterBreakMode(boolean enable) {
    io.setShooterBreakMode(enable);
  }

  public void setShooterMotorPercentSpeed(double percent) {
    io.setShooterMotorPercentSpeed(percent);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
  }
}
