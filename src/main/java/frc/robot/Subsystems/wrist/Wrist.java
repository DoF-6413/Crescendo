// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** shuffleboard tabs: wrist */
  private final ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist");

  private GenericEntry wristkp;
  private GenericEntry wristki;
  private GenericEntry wristkd;
  private GenericEntry wristSetpointSetter;

  /** utb intake pid controller */
  private final PIDController wristPIDController;

  private double wristSetpoint = 0.0;

  /** creates a new wrist, the second joint of the arm subsystem */
  public Wrist(WristIO io) {
    System.out.println("[Init] Creating wrist");
    this.io = io;

    /** creates a new PIDController for the wrist */
    wristPIDController = new PIDController(WristConstants.KP, WristConstants.KI, WristConstants.KD);
    wristPIDController.setSetpoint(wristSetpoint);
    wristPIDController.setTolerance(WristConstants.TOLERANCE_PERCENT * wristSetpoint);

    /** disables continuous input */
    wristPIDController.disableContinuousInput();

    /** Shuffleboard values */
    wristkp = wristTab.add("wristkp", 0.0).getEntry();
    wristki = wristTab.add("wristki", 0.0).getEntry();
    wristkd = wristTab.add("wristkd", 0.0).getEntry();
    wristSetpointSetter = wristTab.add("wristSetpoint", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    /** periodically updates inputs and logs them */
    this.updateInputs();
    Logger.processInputs("Wrist", inputs);

    setWristPercentSpeed(wristPIDController.calculate(inputs.wristPositionRad));

    // TODO: delete once PID is finalized
    /** updates PID values if SmartDashboard gets updated */
    if (WristConstants.KP != wristkp.getDouble(0.0)
        || WristConstants.KI != wristki.getDouble(0.0)
        || WristConstants.KD != wristkd.getDouble(0.0)) {
      updatePIDController();
    }

    if (wristSetpoint != wristSetpointSetter.getDouble(0.0)) {
      updateSetpoint();
    }

    // Gets the current PID values that the PID contollers are set to
    SmartDashboard.putNumber("wristError", wristSetpoint - inputs.wristPositionRad);
    SmartDashboard.putNumber("wristCurrentkP", wristPIDController.getP());
    SmartDashboard.putNumber("wristCurrentkI", wristPIDController.getI());
    SmartDashboard.putNumber("wristCurrentkD", wristPIDController.getD());
    SmartDashboard.putNumber("wristCurrentSetpoint", wristPIDController.getSetpoint());
  }

  /** updates PID values if SmartDashboard gets updated */
  public void updatePIDController() {
    WristConstants.KP = wristkp.getDouble(0.0);
    WristConstants.KI = wristki.getDouble(0.0);
    WristConstants.KD = wristkd.getDouble(0.0);

    wristPIDController.setPID(WristConstants.KP, WristConstants.KI, WristConstants.KD);
  }

  /** updates setpoint if SmartDashboard gets updated */
  public void updateSetpoint() {
    wristSetpoint = wristSetpointSetter.getDouble(0.0);
    wristPIDController.setSetpoint(wristSetpoint);
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
   * @param isEnabled boolean for is brake mode true or false
   */
  public void setWristBrakeMode(boolean isEnabled) {
    setWristBrakeMode(isEnabled);
  }
}
