// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import org.littletonrobotics.junction.Logger;

public class Actuator extends SubsystemBase {
  public final ActuatorIO io;
  public final ActuatorIOInputsAutoLogged inputs = new ActuatorIOInputsAutoLogged();
  private static PIDController actuatorPIDController;
  private double setpoint = 0.0;

  /** Creates a new Actuator. */
  public Actuator(ActuatorIO io) {
    System.out.println("[Init] Creating Actuator");
    this.io = io;
    actuatorPIDController = new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
    actuatorPIDController.setSetpoint(setpoint);
    actuatorPIDController.setTolerance(ArmConstants.ARM_TOLERANCE_PERCENT * setpoint);
    actuatorPIDController.disableContinuousInput();

    SmartDashboard.putNumber("actuatorkp", 0.0);
    SmartDashboard.putNumber("actuatorki", 0.0);
    SmartDashboard.putNumber("actuatorkd", 0.0);
    SmartDashboard.putNumber("actuatorSetpoint", 0.0);
  }

  @Override
  public void periodic() {
    this.updateInputs(); // updates the inputs
    Logger.processInputs("Actuator", inputs); // log the inputs
    
    if (ArmConstants.ARM_KP != SmartDashboard.getNumber("actuatorkp", 0.0)
        || ArmConstants.ARM_KI != SmartDashboard.getNumber("actuatorki", 0.0)
        || ArmConstants.ARM_KD != SmartDashboard.getNumber("actuatorkd", 0.0)) {
      updatePIDController();
    }

    if (setpoint != SmartDashboard.getNumber("actuatorSetpoint", 0.0)) {
      updateSetpoint();
    }
    
    // Gets the current PID values that the PID contollers are set to
    SmartDashboard.putNumber("actuatorError", setpoint - inputs.actuatorPositionRad);
    SmartDashboard.putNumber("actuatorCurrentkP", actuatorPIDController.getP());
    SmartDashboard.putNumber("actuatorCurrentkI", actuatorPIDController.getI());
    SmartDashboard.putNumber("actuatorCurrentkD", actuatorPIDController.getD());
    SmartDashboard.putNumber("actuatorCurrentSetpoint", actuatorPIDController.getSetpoint());

    io.setActuatorPercentSpeed(actuatorPIDController.calculate(inputs.actuatorPositionRad));
  }

  public void updatePIDController() {
   ArmConstants.ARM_KP = SmartDashboard.getNumber("actuatorkp", 0.0);
   ArmConstants.ARM_KI = SmartDashboard.getNumber("actuatorki", 0.0);
   ArmConstants.ARM_KD = SmartDashboard.getNumber("actuatorkd", 0.0);
    actuatorPIDController.setPID(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
  }

  public void updateSetpoint() {
    setpoint = SmartDashboard.getNumber("actuatorSetpoint", 0.0);
    actuatorPIDController.setSetpoint(setpoint);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void setActuatorPercentSpeed(double percent) {
    io.setActuatorPercentSpeed(percent);
  }

  public void setActuatorVoltage(double volts) {
    io.setActuatorVoltage(volts);
  }
}
