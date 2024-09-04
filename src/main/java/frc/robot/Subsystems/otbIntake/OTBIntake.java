// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class OTBIntake extends SubsystemBase {
  private final OTBIntakeIO io;
  private final OTBIntakeIOInputsAutoLogged inputs = new OTBIntakeIOInputsAutoLogged();

  /**
   * Runs the real life motor for the Over the Bumper (OTB) Intake with CAN SPARKMAX Speed
   * Contollers and Neo motor
   */
  public OTBIntake(OTBIntakeIO io) {
    System.out.println("[Init] Creating OTB Intake");
    this.io = io;
  }

  /** Periodically updates the inputs and outputs of the OTB Intake */
  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("OTBIntake", inputs);
  }

  /** Updates inputs for the OTB Intake */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the voltage for the OTB Intake
   *
   * @param volts -12 to 12
   */
  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  /**
   * Sets the speed for the OTB Intake
   *
   * @param percent -1 to 1
   */
  public void setPercentSpeed(double percent) {
    io.setPercentSpeed(percent);
  }

  /**
   * Sets the Brake Mode for the OTB Intake
   *
   * <p>Brake means motor holds position, Coast means easy to move
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }
}
