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
  // private final ShuffleboardTab actuatorTab = Shuffleboard.getTab("Actuator");
  // private GenericEntry actuatorkp;
  // private GenericEntry actuatorki;
  // private GenericEntry actuatorkd;
  // private GenericEntry actuatorSetpointSetter;

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
    actuatorPIDController.setTolerance(
        ActuatorConstants.TOLERANCE_PERCENT * actuatorSetpoint + 0.2);
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
    SmartDashboard.putNumber(
        "PID Calculate", actuatorPIDController.calculate(inputs.actuatorPositionRad));
    SmartDashboard.putNumber("Tolerence", actuatorSetpoint * ActuatorConstants.TOLERANCE_PERCENT);
  }
  // Updates Actuator Speed based on PID Control

  // // TODO: Delete once final PID Numbers are Decided
  // if (ActuatorConstants.KP != actuatorkp.getDouble(0.0)
  //     || ActuatorConstants.KI != actuatorki.getDouble(0.0)
  //     || ActuatorConstants.KD != actuatorkd.getDouble(0.0)) {
  //   updatePIDController();
  // }

  // if (actuatorSetpoint != actuatorSetpointSetter.getDouble(0.0)) {
  //   updateSetpoint();
  // }

  // TODO: Make this appear only in "Test" when Final PID Numbers are Selected
  /** Updates the PID Contants for the PID Controller */
  // public void updatePIDController() {
  //   ActuatorConstants.KP = actuatorkp.getDouble(0.0);
  //   ActuatorConstants.KI = actuatorki.getDouble(0.0);
  //   ActuatorConstants.KD = actuatorkd.getDouble(0.0);
  //   actuatorPIDController.setPID(ActuatorConstants.KP, ActuatorConstants.KI,
  // ActuatorConstants.KD);
  // }

  // // TODO: Make this have a setpoint as a parameter and delete smartdashboard getter
  // /** Updates the Position the Actuator is Going To */
  // public void updateSetpoint() {
  //   actuatorSetpoint = actuatorSetpointSetter.getDouble(0.0);
  //   actuatorPIDController.setSetpoint(actuatorSetpoint);
  // }

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

  public void setActuatorSetpoint(double setpoint) {
    actuatorPIDController.setSetpoint(setpoint);
    actuatorPIDController.setTolerance(
        ActuatorConstants.TOLERANCE_PERCENT * actuatorSetpoint + 0.2);
  }

  public void disableActuator() {
    setActuatorVoltage(0);
  }

  public void setCurrentLimit(int current) {
    io.setCurrentLimit(current);
  }

  public double getOutputCurrent() {
    return inputs.actuatorCurrentAmps[0];
  }

  public void zeroPosition() {
    io.zeroPosition();
  }

  public void actuatorPIDEnable(boolean isEnable) {
    actuatorPIDenable = isEnable;
  }
}
