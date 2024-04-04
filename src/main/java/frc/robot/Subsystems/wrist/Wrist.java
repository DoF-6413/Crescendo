// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** Wrist PID controller */
  private final PIDController wristPIDController;

  private SimpleMotorFeedforward wristFeedforward;
  private final ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist");

  private static GenericEntry wristkp, wristki, wristkd, wristks, wristkv, wristka, wristSetpoint;
  private static double setpoint = 0.0;

  /** Creates a new Wrist, the second joint of the arm mechanism */
  public Wrist(WristIO io) {
    System.out.println("[Init] Creating Wrist");
    this.io = io;

    /** Creates a new PID controller for the Wrist */
    wristPIDController = new PIDController(WristConstants.KP, WristConstants.KI, WristConstants.KD);
    wristPIDController.setSetpoint(0);
    wristPIDController.setTolerance(WristConstants.ANGLE_TOLERANCE);
    wristPIDController.disableContinuousInput();

    wristFeedforward =
        new SimpleMotorFeedforward(WristConstants.KS, WristConstants.KV, WristConstants.KA);

    wristkp = wristTab.add("wristkp", 0.0).getEntry();
    wristki = wristTab.add("wristki", 0.0).getEntry();
    wristkd = wristTab.add("wristkd", 0.0).getEntry();
    wristSetpoint = wristTab.add("wristSetpoint", 0.0).getEntry();

    wristks = wristTab.add("wristks", 0.0).getEntry();
    wristkv = wristTab.add("wristkv", 0.0).getEntry();
    wristka = wristTab.add("wristka", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    /** Periodically updates inputs and logs them */
    this.updateInputs();
    Logger.processInputs("Wrist", inputs);
    // setWristPercentSpeed(
    //     wristPIDController.calculate(inputs.wristAbsolutePositionRad)
    //         + (wristFeedforward.calculate(inputs.wristVelocityRadPerSec)
    //             / RobotStateConstants
    //                 .BATTERY_VOLTAGE)); // Feedforward divided by 12 since it returns a voltage

    if (WristConstants.KP != wristkp.getDouble(0.0)
        || WristConstants.KI != wristki.getDouble(0.0)
        || WristConstants.KD != wristkd.getDouble(0.0)) {
      updatePIDController();
    }

    if (WristConstants.KS != wristks.getDouble(0.0)
        || WristConstants.KV != wristkv.getDouble(0.0)
        || WristConstants.KA != wristka.getDouble(0.0)) {
      updateFFController();
    }

    if (setpoint != wristSetpoint.getDouble(0.0)) {
      updateSetpoint();
    }
  }

  /** Updates the set of loggable inputs for the Wrist */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Updates the PID values from Shuffleboard */
  public void updatePIDController() {
    WristConstants.KP = wristkp.getDouble(0.0);
    WristConstants.KI = wristki.getDouble(0.0);
    WristConstants.KD = wristkd.getDouble(0.0);

    wristPIDController.setPID(WristConstants.KP, WristConstants.KI, WristConstants.KD);
  }

  /** Updates the setpoint from Shuffleboard */
  public void updateSetpoint() {
    setpoint = wristSetpoint.getDouble(0.0);
    wristPIDController.setSetpoint(setpoint);
  }

  /** Updates the PID values from Shuffleboard */
  public void updateFFController() {
    WristConstants.KS = wristks.getDouble(0.0);
    WristConstants.KV = wristkv.getDouble(0.0);
    WristConstants.KA = wristka.getDouble(0.0);
    wristFeedforward =
        new SimpleMotorFeedforward(WristConstants.KS, WristConstants.KV, WristConstants.KA);
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
