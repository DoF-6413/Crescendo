// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    System.out.println("[Init] Creating Climber");
    this.io = io;
  }

  /** Periodically updates the inputs and outputs of the Climber */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Climber", inputs);
  }

  /** Updates the inputs for the Climber */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void setBothClimberVoltage(double volts) {
    io.setBothClimberMotorsVoltage(volts);
  }

  public void setLeftClimberVoltage(double volts) {
    io.setLeftClimberMotorVoltage(volts);
  }

  public void setRightClimberVoltage(double volts) {
    io.setRightClimberMotorVoltage(volts);
  }

  public void setBothClimberPercentSpeed(double percent) {
    io.setBothClimberMotorsPercentSpeed(percent);
  }

  public void setLeftClimberPercentSpeed(double percent) {
    io.setLeftClimberMotorPercentSpeed(percent);
  }

  public void setRightClimberPercentSpeed(double percent) {
    io.setRightClimberMotorPercentSpeed(percent);
  }
}
