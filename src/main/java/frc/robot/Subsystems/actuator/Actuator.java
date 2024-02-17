// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Actuator extends SubsystemBase {
  public final ActuatorIO io;
  public final ActuatorIOInputsAutoLogged inputs = new ActuatorIOInputsAutoLogged();
  
  /** Creates a new Actuator. */
  public Actuator(ActuatorIO io) {
    System.out.println("[init] Creating Actuator");
    this.io = io;
  }

  @Override
  public void periodic() {
    this.updateInputs(); // updates the inputs
    Logger.processInputs("Actuator", inputs); // log the inputs
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void setActuatorVoltage(double volts) {
    io.setActuatorVoltage(volts);
  }

  public void setActuatorPercentSpeed(double percent) {
    io.setActuatorPercentSpeed(percent);
  }
}
