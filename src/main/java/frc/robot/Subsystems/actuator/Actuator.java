// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Actuator extends SubsystemBase {
  /** Creates a new Actuator. */
  public static ActuatorIO actuatorIO;

  public static ActuatorIOInputsAutoLogged actuatorInputs = new ActuatorIOInputsAutoLogged();

  public Actuator(ActuatorIO io) {
    System.out.println("[init] Creating Actuator");
    actuatorIO = io;

    // Creates adjustible PID values and setpoints on smartDashboard (for easier testing)
    SmartDashboard.putNumber("actuatorkp", 0.0);
    SmartDashboard.putNumber("actuatorki", 0.0);
    SmartDashboard.putNumber("actuatorkd", 0.0);
    SmartDashboard.putNumber("actuatorSetpoint", 0.0);
  }

  @Override
  public void periodic() {
    actuatorIO.updateInputs(actuatorInputs); // updates the inputs
    Logger.processInputs("Actuator", actuatorInputs); // log the inputs
  }

  /** return the position in meters (just for human understanding bc we use rad) */
  public double getActuatorPosition() {
    return actuatorInputs.actuatorPositionRad * 2 * Math.PI;
  }

  public void setActuatorVoltage(double volts) {
    actuatorIO.setActuatorVoltage(volts);
  }

  public double getActuatorPositionRad() { // same as position but in radians
    return actuatorInputs.actuatorPositionRad;
  }

  public void setActuatorPercentSpeed(double percent) {
    actuatorIO.setActuatorPercentSpeed(percent);
  }
}
