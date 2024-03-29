// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

  // Creates the PID Contollers for both shooter motors
  private final PIDController topShooterPIDController;
  private final PIDController bottomShooterPIDController;

  // The desired RPM for the shooter
  private double setpointRPM = 0.0;

  public Shooter(ShooterIO io) {

    System.out.println("[Init] Creating Shooter");
    this.io = io;

    topShooterPIDController =
        new PIDController(
            ShooterConstants.TOP_KP, ShooterConstants.TOP_KI, ShooterConstants.TOP_KD);
    bottomShooterPIDController =
        new PIDController(
            ShooterConstants.BOTTOM_KP, ShooterConstants.BOTTOM_KI, ShooterConstants.BOTTOM_KD);

    topShooterPIDController.setSetpoint(setpointRPM);
    bottomShooterPIDController.setSetpoint(setpointRPM);

    // Sets the tolerance of the setpoint
    topShooterPIDController.setTolerance(ShooterConstants.RPM_TOLERANCE);
    bottomShooterPIDController.setTolerance(ShooterConstants.RPM_TOLERANCE);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);

    // Sets the voltage of the Shooter Motors using PID
    setTopShooterMotorVoltage(
        topShooterPIDController.calculateForVoltage(
            inputs.topShooterMotorRPM, ShooterConstants.MAX_RPM));
    setBottomShooterMotorVoltage(
        bottomShooterPIDController.calculateForVoltage(
            inputs.bottomShooterMotorRPM, ShooterConstants.MAX_RPM));

    // SmartDashboard.putNumber("ShooterTopSetpoint", topShooterPIDController.getSetpoint());
    // SmartDashboard.putNumber("ShooterBottomSetpoint", bottomShooterPIDController.getSetpoint());
    SmartDashboard.putBoolean("BothAtSetpoint", allAtSetpoint());
    // SmartDashboard.putBoolean("TopAtSetpoint", topAtSetpoint());
    // SmartDashboard.putBoolean("BottomAtSetpoint", bottomAtSetpoint());

  }

  /** Updates the set of loggable inputs for both Shooter Motors */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the Brake Mode for the Shooter (Brake means motor holds position, Coast means easy to
   * move)
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Sets BOTH Shooter Motors at a percentage of its max speed.
   *
   * <p>A positve number spins the Top Shooter Motor CCW and the Bottom Shooter Motor CW and vice
   * versa for a negative number
   *
   * @param percent -1 to 1
   */
  public void setShooterMotorPercentSpeed(double percent) {
    io.setBothShooterMotorPercentSpeed(percent);
  }

  /**
   * Sets BOTH Shooter Motors at the specified Voltage
   *
   * @param volts -12 to 12
   */
  public void setBothShooterMotorsVoltage(double volts) {
    io.setBothShooterMotorsVoltage(volts);
  }

  /**
   * Sets the voltage of the Top Shooter Motor
   *
   * @param volts -12 to 12
   */
  public void setTopShooterMotorVoltage(double volts) {
    io.setTopShooterMotorVoltage(volts);
  }

  /**
   * Sets the voltage of the Bottom Shooter Motor
   *
   * @param volts -12 to 12
   */
  public void setBottomShooterMotorVoltage(double volts) {
    io.setBottomShooterMotorVoltage(volts);
  }

  /** Returns where the Top Shooter RPM is within the setpoint, including tolerance */
  public boolean topAtSetpoint() {
    return topShooterPIDController.atSetpoint(inputs.topShooterMotorRPM);
  }

  /** Returns where the Bottom Shooter RPM is within the setpoint, including tolerance */
  public boolean bottomAtSetpoint() {
    return bottomShooterPIDController.atSetpoint(inputs.bottomShooterMotorRPM);
  }

  /** Returns whether BOTH Shooter motors are at their setpoint */
  public boolean allAtSetpoint() {
    return bottomAtSetpoint() && topAtSetpoint();
  }

  /**
   * Sets the PID setpoint of the Shooter
   *
   * @param setpoint RPM
   */
  public void setSetpoint(double setpoint) {
    topShooterPIDController.setSetpoint(setpoint);
    bottomShooterPIDController.setSetpoint(setpoint);
  }

  public void setTolerance(double tolerance) {
    topShooterPIDController.setTolerance(tolerance);
    bottomShooterPIDController.setTolerance(tolerance);
  }
}
