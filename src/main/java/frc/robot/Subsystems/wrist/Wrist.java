// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** Wrist PID controller */
  private final PIDController wristPIDController;

  /** Creates a new Wrist, the second joint of the arm mechanism */
  public Wrist(WristIO io) {
    System.out.println("[Init] Creating Wrist");
    this.io = io;

    /** Creates a new PID controller for the Wrist */
    wristPIDController = new PIDController(WristConstants.KP, WristConstants.KI, WristConstants.KD);
    wristPIDController.setSetpoint(0);
    wristPIDController.setTolerance(WristConstants.ANGLE_TOLERANCE);
    wristPIDController.disableContinuousInput();
  }

  @Override
  public void periodic() {
    /** Periodically updates inputs and logs them */
    this.updateInputs();
    Logger.processInputs("Wrist", inputs);

    // setWristPercentSpeed(wristPIDController.calculate(inputs.wristAbsolutePositionRad));
  }

  /** Updates the set of loggable inputs for the Wrist */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets Wrist to a percentage of its maximum speed
   *
   * @param percent -1 to 1
   */
  public void setWristPercentSpeed(double percent) {
    io.setWristPercentSpeed(percent);
  }

  /**
   * Sets voltage of the Wrist
   *
   * @param volts -12 to 12
   */
  public void setWristMotorVoltage(double volts) {
    setWristMotorVoltage(volts);
  }

  /**
   * Sets brake mode of the Wrist
   *
   * @param enable Sets brake mode if true, coast if false
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Updates the angle that the Wrist should be at using the WPI PID controller
   *
   * @param setpoint Angle (Radians)
   */
  public void setSetpoint(double setpoint) {
    wristPIDController.setSetpoint(setpoint);
  }

  /** Returns whether the Wrist is at it's setpoint or not */
  public boolean atSetpoint() {
    return wristPIDController.atSetpoint();
  }

  /**
   * Changes the angle setpoint of the Wrist
   *
   * @param increment Angle (Radians)
   */
  public void incrementWristSetpoint(double increment) {
    wristPIDController.setSetpoint(wristPIDController.getSetpoint() + increment);
  }
}
