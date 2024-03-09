// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Actuator extends SubsystemBase {
  public final ActuatorIO io;
  public final ActuatorIOInputsAutoLogged inputs = new ActuatorIOInputsAutoLogged();
  private final PIDController actuatorPIDController;
  private double actuatorSetpoint = 0.0;
  private boolean actuatorPIDenable = true;

  /**
   * Creates a new Actuator, the Subsystem that moves the OTB Intake Rollers from inside the Robot
   * Frame to Outside the Robot
   */
  public Actuator(ActuatorIO io) {
    System.out.println("[Init] Creating Actuator");
    this.io = io;
    actuatorPIDController =
        new PIDController(ActuatorConstants.KP, ActuatorConstants.KI, ActuatorConstants.KD);
    actuatorPIDController.setSetpoint(actuatorSetpoint);
    actuatorPIDController.setTolerance(ActuatorConstants.ANGLE_TOLERANCE);
    actuatorPIDController.disableContinuousInput();
  }

  @Override
  public void periodic() {
    this.updateInputs();
    // log the inputs
    Logger.processInputs("Actuator", inputs);
    SmartDashboard.putNumber("Setpoint", actuatorSetpoint);

    if (actuatorPIDenable) {
      setActuatorPercentSpeed(actuatorPIDController.calculate(inputs.actuatorPositionRad));
    }
  }

  /** Updates the Outputs of the Motors based on What Mode we are In */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the Actuator motor to a percentage of its max speed
   *
   * @param percent -1 to 1
   */
  public void setActuatorPercentSpeed(double percent) {
    io.setActuatorPercentSpeed(percent);
  }

  /**
   * Sets the voltage of the Actuator motor
   *
   * @param volts -12 to 12
   */
  public void setActuatorVoltage(double volts) {
    io.setActuatorVoltage(volts);
  }

  /**
   * Sets the Brake Mode for the Actuator (Brake means motor holds position, Coast means easy to
   * move)
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Sets the PID setpoint of the Actuator
   * 
   * @param setpoint Angle (Radians)
   */
  public void setActuatorSetpoint(double setpoint) {
    actuatorPIDController.setSetpoint(setpoint);
  }

  /**
   * Sets the smart current limiting of the Actuator using the SPARK MAX speed contollers
   *
   * @param current Amps
   */
  public void setCurrentLimit(int current) {
    io.setCurrentLimit(current);
  }

  /**
   * @return The current ouputs of the Actuator in amps
   */
  public double getOutputCurrent() {
    return inputs.actuatorCurrentAmps[0];
  }

  /** Resets the current position of the Actuator to be the new zero position */
  public void zeroPosition() {
    io.zeroPosition();
  }

  /**
   * Enables or disables the position PID calculations of the Actuator
   *
   * @param isEnable True enables PID, false disables PID
   */
  public void actuatorPIDEnable(boolean isEnable) {
    actuatorPIDenable = isEnable;
  }
}
