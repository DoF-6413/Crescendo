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
    actuatorPIDController.setTolerance(ActuatorConstants.TOLERANCE_PERCENT * actuatorSetpoint);
    actuatorPIDController.disableContinuousInput();

    // TODO: Delete once final PID Numbers are Decided
    SmartDashboard.putNumber("actuatorkp", 0.0);
    SmartDashboard.putNumber("actuatorki", 0.0);
    SmartDashboard.putNumber("actuatorkd", 0.0);
    SmartDashboard.putNumber("actuatorSetpoint", 0.0);
  }

  @Override
  public void periodic() {
    // updates the inputs
    this.updateInputs();
    // log the inputs
    Logger.processInputs("Actuator", inputs);

    // Updates Actuator Speed based on PID Control
    setActuatorPercentSpeed(actuatorPIDController.calculate(inputs.actuatorPositionRad));

    // TODO: Delete once final PID Numbers are Decided
    if (ActuatorConstants.KP != SmartDashboard.getNumber("actuatorkp", 0.0)
        || ActuatorConstants.KI != SmartDashboard.getNumber("actuatorki", 0.0)
        || ActuatorConstants.KD != SmartDashboard.getNumber("actuatorkd", 0.0)) {
      updatePIDController();
    }

    if (actuatorSetpoint != SmartDashboard.getNumber("actuatorSetpoint", 0.0)) {
      updateSetpoint();
    }

    // Gets the current PID values that the PID contollers are set to
    SmartDashboard.putNumber("actuatorError", actuatorSetpoint - inputs.actuatorPositionRad);
    SmartDashboard.putNumber("actuatorCurrentkP", actuatorPIDController.getP());
    SmartDashboard.putNumber("actuatorCurrentkI", actuatorPIDController.getI());
    SmartDashboard.putNumber("actuatorCurrentkD", actuatorPIDController.getD());
    SmartDashboard.putNumber("actuatorCurrentSetpoint", actuatorPIDController.getSetpoint());
  }

  // TODO: Make this appear only in "Test" when Final PID Numbers are Selected
  /** Updates the PID Contants for the PID Controller */
  public void updatePIDController() {
    ActuatorConstants.KP = SmartDashboard.getNumber("actuatorkp", 0.0);
    ActuatorConstants.KI = SmartDashboard.getNumber("actuatorki", 0.0);
    ActuatorConstants.KD = SmartDashboard.getNumber("actuatorkd", 0.0);
    actuatorPIDController.setPID(ActuatorConstants.KP, ActuatorConstants.KI, ActuatorConstants.KD);
  }

  // TODO: Make this have a setpoint as a parameter and delete smartdashboard getter
  /** Updates the Position the Actuator is Going To */
  public void updateSetpoint() {
    actuatorSetpoint = SmartDashboard.getNumber("actuatorSetpoint", 0.0);
    actuatorPIDController.setSetpoint(actuatorSetpoint);
  }

  /** Updates the Outputs of the Motors based on What Mode we are In */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the Actuator motor to a percentage of its max speed
   *
   * @param percent [-1 to 1]
   */
  public void setActuatorPercentSpeed(double percent) {
    io.setActuatorPercentSpeed(percent);
  }

  /**
   * Sets the voltage of the Actuator motor
   *
   * @param volts [-12 to 12]
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
}
