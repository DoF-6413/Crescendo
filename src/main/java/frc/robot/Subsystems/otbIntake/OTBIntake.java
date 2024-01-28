// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Runs the motor for the Over the Bumper (OTB) Intake */
public class OTBIntake extends SubsystemBase {

  private final OTBIntakeIO io;
  private final OTBIntakeIOInputsAutoLogged inputs = new OTBIntakeIOInputsAutoLogged();

  public OTBIntake(OTBIntakeIO io) {
    System.out.println("[Init] Creating OTB Intake");
    this.io = io;
  }

  /** Periodically updates the inputs and outputs of the OTB Intake */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("OTBIntake", inputs);
  }

  /** Updates inputs for the OTB Intake */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Sets intake voltage for the OTB Intake */
  public void setOTBIntakeVoltage(double volts) {
    io.setOTBIntakeVoltage(volts);
  }

  /** Sets intake speed for the OTB Intake */
  public void setOTBIntakePercentSpeed(double percent) {
    io.setOTBIntakePercentSpeed(percent);
  }
}
