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
  private double actuatorSetpoint = 0.0;

  public Actuator(ActuatorIO io) {
    System.out.println("[init] creating Actuator");
    actuatorIO = io;
    actuatorPID =
        new PIDController(
            ActuatorConstants.ACTUATOR_KP,
            ActuatorConstants.ACTUATOR_KI,
            ActuatorConstants.ACTUATOR_KD);

    actuatorPID.setSetpoint(actuatorSetpoint);

    // This sets constraints on the rot of the actuator (takes shortest path between setpoints so it
    // doesn't go all the way around)
    actuatorPID.enableContinuousInput(
        ActuatorConstants.ACTUATOR_MIN_SETPOINT, ActuatorConstants.ACTUATOR_MIN_SETPOINT);

    actuatorPID.setTolerance(actuatorSetpoint * ActuatorConstants.ACTUATOR_TOLERANCE);

    // Creates adjustible PID values and setpoints on smartDashboard (for easier testing)
    SmartDashboard.putNumber("actuatorkp", 0.0);
    SmartDashboard.putNumber("actuatorki", 0.0);
    SmartDashboard.putNumber("actuatorkd", 0.0);

    SmartDashboard.putNumber("actuatorSetpoint", 0.0);
  }

  @Override
  public void periodic() {
    actuatorIO.updateInputs(actuatorInputs); // updates the inputs
    Logger.processInputs("Actuator", actuatorInputs); // logg the inputs

    // This updates the PID values that are put in the SmartDashboard
    if (ActuatorConstants.ACTUATOR_KP != SmartDashboard.getNumber("actuatorkp", 0)
        || ActuatorConstants.ACTUATOR_KI != SmartDashboard.getNumber("actuatorki", 0)
        || ActuatorConstants.ACTUATOR_KD != SmartDashboard.getNumber("actuatorkd", 0)) {
      updatePIDController();
    }

    // This updates the setpoint if changed in the SmartDashboard
    if (actuatorSetpoint != SmartDashboard.getNumber("actuatorSetpoint", 0.0)) {
      updateSetpoint();
    }

    // For testing- making sure you are getting the values you are giving it
    SmartDashboard.putNumber("Actuator setpoint", actuatorSetpoint);
    SmartDashboard.putNumber("ActuatorError", actuatorSetpoint - getActuatorPosition());

    actuatorIO.setActuatorPosition(actuatorPID.calculate(getActuatorPositionRad()));
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

  public double
      getActuatorPosition() { // return the position in meters (just for human understanding bc we
    // use rad)
    return actuatorInputs.turnPositionRad * 2 * Math.PI;
  }

  public void setActuatorSpeed(double voltage) {
    actuatorIO.setActuatorSpeed(voltage);
  }

  public double getActuatorPositionRad() { // same as position but in radians
    return actuatorInputs.turnPositionRad;
  }

  public void setActuatorPercentSpeed(double percent) {
    actuatorIO.setActuatorSpeed(
        percent * 12); // sets the speed based on a percentage not just voltge
  }

  private void updateSetpoint() {
    actuatorSetpoint = SmartDashboard.getNumber("actuatorSetpoint", 0.0);
    actuatorPID.setSetpoint(actuatorSetpoint);
  }
}
