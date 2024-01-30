// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private static PIDController topShooterPID;
  private static PIDController bottomShooterPID;
  // private SimpleMotorFeedforward shooterFF =
  // new SimpleMotorFeedforward(
  // ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV,
  // ShooterConstants.SHOOTER_KA);
  private double setpointRPM = 0;

  // TODO: delete
  private double shooterkp = 0;
  private double shooterki = 0;
  private double shooterkd = 0;

  public Shooter(ShooterIO io) {

    topShooterPID =
        new PIDController(
            ShooterConstants.SHOOTER_KP, ShooterConstants.SHOOTER_KI, ShooterConstants.SHOOTER_KD);

    bottomShooterPID =
        new PIDController(
            ShooterConstants.SHOOTER_KP, ShooterConstants.SHOOTER_KI, ShooterConstants.SHOOTER_KD);

    System.out.println("[Init] Creating Shooter");
    this.io = io;
    topShooterPID.setTolerance(
        setpointRPM * 0.05); // The RPM of the motors can be within 50 RPM of the goal
    bottomShooterPID.setTolerance(
        setpointRPM * 0.05); // The RPM of the motors can be within 50 RPM of the goal
    SmartDashboard.putNumber("shooterkp", 0);
    SmartDashboard.putNumber("shooterki", 0);
    SmartDashboard.putNumber("shooterkd", 0);
    SmartDashboard.putNumber("setpoint", 0);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);
    io.setTopShooterMotorVoltage(topShooterPID.calculate(getTopRPM(), setpointRPM));
    io.setBottomShooterMotorVoltage(bottomShooterPID.calculate(getBottomRPM(), setpointRPM));
    if (shooterkp != SmartDashboard.getNumber("shooterkp", 0)
        || shooterki != SmartDashboard.getNumber("shooterki", 0)
        || shooterkd != SmartDashboard.getNumber("shooterkd", 0)) {
      updatePIDController();
    }

    updateSetpoint(SmartDashboard.getNumber("setpoint", 0));

    SmartDashboard.putNumber("Setpoint Put Number", setpointRPM);

    SmartDashboard.putNumber("TopMotorError", setpointRPM - getTopRPM());
    SmartDashboard.putNumber("TopBottomError", setpointRPM - getBottomRPM());
  }

  public void updatePIDController() {
    shooterkp = SmartDashboard.getNumber("shooterkp", 0);
    shooterki = SmartDashboard.getNumber("shooterki", 0);
    shooterkd = SmartDashboard.getNumber("shooterkd", 0);
    topShooterPID = new PIDController(shooterkp, shooterki, shooterkd);
    bottomShooterPID = new PIDController(shooterkp, shooterki, shooterkd);
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
    return inputs.topShooterMotorRPM;
  }

  public double getBottomRPM() {
    return inputs.bottomShooterMotorRPM;
  }

  public void updateSetpoint(double newSetpoint) {
    setpointRPM = newSetpoint;
  }

  // TODO: One function for Top and Bottom
  // public boolean atSetpoint() {
  //   return shooterPIDController.atSetpoint();
  // }
}
