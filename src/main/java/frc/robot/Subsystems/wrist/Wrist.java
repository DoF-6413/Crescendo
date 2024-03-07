// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** Wrist pid controller */
  private final PIDController wristPIDController;

  /** Creates a new wrist, the second joint of the arm subsystem */
  public Wrist(WristIO io) {
    System.out.println("[Init] Creating wrist");
    this.io = io;

    /** Creates a new PIDController for the wrist */
    wristPIDController = new PIDController(WristConstants.KP, WristConstants.KI, WristConstants.KD);
    wristPIDController.setSetpoint(0);
    wristPIDController.setTolerance(Units.degreesToRadians(1));
    wristPIDController.disableContinuousInput();
  }

  @Override
  public void periodic() {
    /** periodically updates inputs and logs them */
    this.updateInputs();
    Logger.processInputs("Wrist", inputs);

    setWristPercentSpeed(wristPIDController.calculate(inputs.wristAbsolutePositionRad));
  }

  /** updates setpoint if SmartDashboard gets updated */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets Wrist Percent Speed
   *
   * @param percent [-1 to 1]
   */
  public void setWristPercentSpeed(double percent) {
    io.setWristPercentSpeed(percent);
  }

  /**
   * Sets Wrist Voltage
   *
   * @param volts [-12 to 12]
   */
  public void setWristMotorVoltage(double volts) {
    setWristMotorVoltage(volts);
  }

  /**
   * Sets brake mode
   *
   * @param enable boolean for is brake mode true or false
   */
  public void setWristBrakeMode(boolean enable) {
    io.setWristBrakeMode(enable);
  }

  /**
   * Updates the angle that the wrist should be at using the WPI PID controller
   *
   * @param setpoint Radians [RANGE]
   */
  public void setSetpoint(double setpoint) {
    wristPIDController.setSetpoint(setpoint);
  }

  /** Returns whether the wrist is at it's setpoint or not */
  public boolean atSetpoint() {
    return wristPIDController.atSetpoint();
  }

  public void incrementWristSetpoint(double increment) {
    wristPIDController.setSetpoint(wristPIDController.getSetpoint() + increment);
  }

  /**
   * Sets the wrist to a specified angle
   *
   * @param double angle in radians
   */
  public void setWristSetpoint(double setpoint) {
    wristPIDController.setSetpoint(setpoint);
  }
}
