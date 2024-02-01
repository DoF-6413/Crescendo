// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private static PIDController topShooterPID;
  private static PIDController bottomShooterPID;

  private double setpointRPM = 0.0;

  // TODO: delete
  private double topShooterkp = 0.0;
  private double topShooterki = 0.0;
  private double topShooterkd = 0.0;
  private double bottomShooterkp = 0.0;
  private double bottomShooterki = 0.0;
  private double bottomShooterkd = 0.0;

  public Shooter(ShooterIO io) {

    topShooterPID = new PIDController(topShooterkp, topShooterki, topShooterkd);
    bottomShooterPID = new PIDController(bottomShooterkp, bottomShooterki, bottomShooterkd);
    topShooterPID.setSetpoint(setpointRPM);
    topShooterPID.disableContinuousInput();

    System.out.println("[Init] Creating Shooter");
    this.io = io;
    SmartDashboard.putNumber("topShooterkp", 0);
    SmartDashboard.putNumber("topShooterki", 0);
    SmartDashboard.putNumber("topShooterkd", 0);
    SmartDashboard.putNumber("bottomShooterkp", 0);
    SmartDashboard.putNumber("bottomShooterki", 0);
    SmartDashboard.putNumber("bottomShooterkd", 0);
    SmartDashboard.putNumber("setpoint", 0);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);
    if (topShooterkp != SmartDashboard.getNumber("topShooterkp", 0)
        || topShooterki != SmartDashboard.getNumber("topShooterki", 0)
        || topShooterkd != SmartDashboard.getNumber("topShooterkd", 0)
        || bottomShooterkp != SmartDashboard.getNumber("bottomShooterkp", 0)
        || bottomShooterki != SmartDashboard.getNumber("bottomShooterki", 0)
        || bottomShooterkd != SmartDashboard.getNumber("bottomShooterkd", 0)) {
      updatePIDController();
    }

    topShooterPID.setTolerance(200); // The RPM of the motors can be within 50 RPM of the goal
    bottomShooterPID.setTolerance(
        setpointRPM * 0.05); // The RPM of the motors can be within 5% of the setpoint

    if (setpointRPM != SmartDashboard.getNumber("setpoint", 0)) {
      updateSetpoint();
    }

    SmartDashboard.putNumber("Setpoint Put Number", setpointRPM);

    SmartDashboard.putNumber("TopMotorError", setpointRPM - getTopRPM());
    SmartDashboard.putNumber("BottomMotorError", setpointRPM - getBottomRPM()); // bottommotorerror

    io.setTopShooterMotorVoltage(topShooterPID.calculate(getTopRPM(), setpointRPM));
    io.setBottomShooterMotorVoltage(bottomShooterPID.calculate(getBottomRPM(), setpointRPM));
  }

  public void updatePIDController() {
    topShooterkp = SmartDashboard.getNumber("topShooterkp", 0);
    topShooterki = SmartDashboard.getNumber("topShooterki", 0);
    topShooterkd = SmartDashboard.getNumber("topShooterkd", 0);
    bottomShooterkp = SmartDashboard.getNumber("bottomShooterkp", 0);
    bottomShooterki = SmartDashboard.getNumber("bottomShooterki", 0);
    bottomShooterkd = SmartDashboard.getNumber("bottomShooterkd", 0);
    topShooterPID = new PIDController(topShooterkp, topShooterki, topShooterkd);
    bottomShooterPID = new PIDController(bottomShooterkp, bottomShooterki, bottomShooterkd);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void setShooterMotorsVoltage(double volts) {
    io.setBothShooterMotorsVoltage(volts);
  }

  public void setShooterBreakMode(boolean enable) {
    io.setShooterBreakMode(enable);
  }

  public void setShooterMotorPercentSpeed(double percent) {
    io.setBothShooterMotorPercentSpeed(percent);
  }

  public double getTopRPM() {
    return Math.abs(inputs.topShooterMotorRPM);
  }

  public double getBottomRPM() {
    return Math.abs(inputs.bottomShooterMotorRPM);
  }

  public void updateSetpoint() {
    setpointRPM = SmartDashboard.getNumber("setpoint", 0);
    topShooterPID.setSetpoint(setpointRPM);
  }

  // TODO: One function for Top and Bottom
  // public boolean atSetpoint() {
  //   return shooterPIDController.atSetpoint();
  // }
}
