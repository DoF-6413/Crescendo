// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private final PIDController feederPIDController;
  private double setpointRPM = 0.0;

  /** Creates a new Feeder */
  public Feeder(FeederIO io) {
    System.out.println("[Init] Creating Feeder");
    this.io = io;
    feederPIDController =
        new PIDController(FeederConstants.KP, FeederConstants.KI, FeederConstants.KD);
    feederPIDController.setSetpoint(setpointRPM);
    feederPIDController.setTolerance(setpointRPM * FeederConstants.TOLERANCE_PERCENT);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Feeder", inputs);

    setFeederVoltage(
        feederPIDController.calculateForVoltage(inputs.feederRPM, FeederConstants.MAX_RPM));
  }

  /** Updates the set of loggable inputs for both Shooter Motors */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the voltage of the Feeder motor
   *
   * @param volts -12 to 12
   */
  public void setFeederVoltage(double volts) {
    io.setFeederVoltage(volts);
  }

  /**
   * Sets the speed of the Feeder motor based on a percent of its maximum speed
   *
   * @param percent -1 to 1
   */
  public void setFeederPercentSpeed(double percent) {
    io.setFeederPercentSpeed(percent);
  }

  /**
   * Sets the Feeder motor to brake mode
   *
   * @param enable
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Sets the PID setpoint of the Feeder
   *
   * @param setpoint RPM
   */
  public void setSetpoint(double setpoint) {
    feederPIDController.setSetpoint(setpoint);
  }
}
