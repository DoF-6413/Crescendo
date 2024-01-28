// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuatorConstants;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;


public class Actuator extends SubsystemBase {
  /** Creates a new Actuator. */
  public static ActuatorIO actuatorIO;

  public static ActuatorIOInputsAutoLogged actuatorInputs = new ActuatorIOInputsAutoLogged();
  public static PIDController actuatorPID;
  private double actuatorSetpoint = 0.0;

  public Actuator(ActuatorIO io) {
    System.out.println("[init] Creating Actuator");
    actuatorIO = io;
    actuatorPID =
        new PIDController(
            ActuatorConstants.ACTUATOR_KP,
            ActuatorConstants.ACTUATOR_KI,
            ActuatorConstants.ACTUATOR_KD);

    actuatorPID.setSetpoint(actuatorSetpoint);

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

    actuatorIO.setActuatorVoltage(
        actuatorPID.calculateForVoltage(getActuatorPositionRad(),
    ActuatorConstants.ACTUATOR_MAX_ANGLE_RADS));
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

  /** return the position in meters (just for human understanding bc we use rad) */
  public double getActuatorPosition() {
    return actuatorInputs.actuatorPositionRad * 2 * Math.PI;
  }

  public void setActuatorVoltage(double voltage) {
    actuatorIO.setActuatorVoltage(voltage);
  }

  public double getActuatorPositionRad() { // same as position but in radians
    return actuatorInputs.actuatorPositionRad;
  }

  /** Sets the speed based on a percentage not just voltge */
  public void setActuatorPercentSpeed(double percent) {
    actuatorIO.setActuatorPercentSpeed(percent);
  }

  private void updateSetpoint() {
    actuatorSetpoint = SmartDashboard.getNumber("actuatorSetpoint", 0.0);
    actuatorPID.setSetpoint(actuatorSetpoint);
  }
