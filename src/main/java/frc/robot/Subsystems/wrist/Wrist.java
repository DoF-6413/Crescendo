// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private static PIDController wristPIDController;
  private double wristSetpoint = 0.0;

  /** creates a new wrist, the second joint of the arm subsystem */
  public Wrist(WristIO io) {
    System.out.println("[Init] Creating wrist");
    this.io = io;

    wristPIDController =
        new PIDController(
            WristConstants.KP, WristConstants.KI, WristConstants.KD);
    wristPIDController.setSetpoint(wristSetpoint);
    wristPIDController.setTolerance(WristConstants.TOLERANCE_PERCENT * wristSetpoint);
    wristPIDController.disableContinuousInput();

    SmartDashboard.putNumber("wristkp", 0.0);
    SmartDashboard.putNumber("wristki", 0.0);
    SmartDashboard.putNumber("wristkd", 0.0);
    SmartDashboard.putNumber("wristSetpoint", 0.0);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Wrist", inputs);

    if (WristConstants.KP != SmartDashboard.getNumber("wristkp", 0.0)
        || WristConstants.KI != SmartDashboard.getNumber("wristki", 0.0)
        || WristConstants.KD != SmartDashboard.getNumber("wristkd", 0.0)) {
      updatePIDController();
    }

    if (wristSetpoint != SmartDashboard.getNumber("wristSetpoint", 0.0)) {
      updateSetpoint();
    }

    // Gets the current PID values that the PID contollers are set to
    SmartDashboard.putNumber("wristError", wristSetpoint - inputs.wristPositionRad);
    SmartDashboard.putNumber("wristCurrentkP", wristPIDController.getP());
    SmartDashboard.putNumber("wristCurrentkI", wristPIDController.getI());
    SmartDashboard.putNumber("wristCurrentkD", wristPIDController.getD());
    SmartDashboard.putNumber("wristCurrentSetpoint", wristPIDController.getSetpoint());

    io.setWristPercentSpeed(wristPIDController.calculate(inputs.wristPositionRad));
  }

  public void updatePIDController() {
    WristConstants.KP = SmartDashboard.getNumber("wristkp", 0.0);
    WristConstants.KI = SmartDashboard.getNumber("wristki", 0.0);
    WristConstants.KD = SmartDashboard.getNumber("wristkd", 0.0);
    wristPIDController.setPID(
        WristConstants.KP, WristConstants.KI, WristConstants.KD);
  }

  public void updateSetpoint() {
    wristSetpoint = SmartDashboard.getNumber("wristSetpoint", 0.0);
    wristPIDController.setSetpoint(wristSetpoint);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Sets speed for the wrist */
  public void setWristPercentSpeed(double percent) {
    io.setWristPercentSpeed(percent);
  }

  /** Sets voltage for the wrist */
  public void setWristMotorVoltage(double volts) {
    setWristMotorVoltage(volts);
  }
}
