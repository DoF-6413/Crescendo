// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuatorConstants;
import org.littletonrobotics.junction.Logger;

public class Actuator extends SubsystemBase {
  /** Creates a new Actuator. */
  public static ActuatorIO actuatorIO;

  public static ActuatorIOInputsAutoLogged actuatorInputs;
  public static PIDController actuatorPID;
  private double actuatorSetpoint;

  public Actuator(ActuatorIO io) {
    System.out.println("[init] creating Actuator");
    actuatorIO = io;
    actuatorPID =
        new PIDController(
            ActuatorConstants.ACTUATOR_KP,
            ActuatorConstants.ACTUATOR_KI,
            ActuatorConstants.ACTUATOR_KD);
    actuatorPID.setTolerance(actuatorSetpoint * ActuatorConstants.ACTUATOR_TOLERANCE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    actuatorIO.updateInputs(actuatorInputs); // update the inputs
    Logger.processInputs("Actuator", actuatorInputs); // logg the inputs
    actuatorIO.setActuatorSpeed(setpointRPM);
    actuatorIO.setActuatorVoltage(actuatorPID.calculate(getActuatorPositionRad(), ));   //TODO: fix this to match actuator position instead of RPM 
    if (ActuatorConstants.ACTUATOR_KP != SmartDashboard.getNumber("actuatorkp", 0)
        || ActuatorConstants.ACTUATOR_KI != SmartDashboard.getNumber("actuatorki", 0)
        || ActuatorConstants.ACTUATOR_KD != SmartDashboard.getNumber("actuatorkd", 0)) {
      updatePIDController();
    }

    updateSetpoint(SmartDashboard.getNumber("setpoint", 0));

    SmartDashboard.putNumber("Actuator setpoint", actuatorSetpoint);
    SmartDashboard.putNumber("ActuatorError", actuatorSetpoint - getActuatorPosition());
  }

  private void updatePIDController() {
    ActuatorConstants.ACTUATOR_KP = SmartDashboard.getNumber("actuatorkp", 0);
    ActuatorConstants.ACTUATOR_KI = SmartDashboard.getNumber("actuatorki", 0);
    ActuatorConstants.ACTUATOR_KD = SmartDashboard.getNumber("actuatorkd", 0);
    actuatorPID =
        new PIDController(
            ActuatorConstants.ACTUATOR_KP,
            ActuatorConstants.ACTUATOR_KI,
            ActuatorConstants.ACTUATOR_KD);
  }

  public double getActuatorPosition() { // return the position in meters
    return actuatorInputs.turnPositionRad * 2 * Math.PI;
  }

  public void setActuatorSpeed(double voltage) {
    actuatorIO.setActuatorSpeed(voltage);
  }

  public double getActuatorPositionRad() {
    return actuatorInputs.turnPositionRad;
  }

  public void setActuatorPercentSpeed(double percent) {
    actuatorIO.setActuatorSpeed(
        percent * 12); // sets the speed based on a percentage not just voltge
  }

  private void updateSetpoint(double newSetpoint) {
    actuatorSetpoint = newSetpoint;
  }
}
