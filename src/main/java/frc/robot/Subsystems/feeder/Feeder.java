// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  // Creates adjustable PID values on a Shuffleboard tab
  // private final ShuffleboardTab feederTab = Shuffleboard.getTab("Feeder");
  // private GenericEntry feederkP;
  // private GenericEntry feederkI;
  // private GenericEntry feederkD;
  // private GenericEntry feederSetpointSetter;
  private double setpointRPM = 0.0;
  private final PIDController feederPIDController;

  /** Creates a new Feeder */
  public Feeder(FeederIO io) {
    System.out.println("[Init] Creating Feeder");
    this.io = io;

    feederPIDController =
        new PIDController(FeederConstants.KP, FeederConstants.KI, FeederConstants.KD);
    feederPIDController.setSetpoint(setpointRPM);
    feederPIDController.setTolerance(setpointRPM * FeederConstants.TOLERANCE_PERCENT);

    // Puts adjustable PID values and setpoints onto the SmartDashboard
    // feederkP = feederTab.add("feederkP", 0.0).getEntry();
    // feederkI = feederTab.add("feederkI", 0.0).getEntry();
    // feederkD = feederTab.add("feederkD", 0.0).getEntry();
    // feederSetpointSetter = feederTab.add("feederSetpoint", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Feeder", inputs);

    setFeederVoltage(
        feederPIDController.calculateForVoltage(inputs.feederRPM, FeederConstants.MAX_VALUE));

    // if (setpointRPM != feederSetpointSetter.getDouble(0.0)) {
    //   updateSetpoint();
    // }
  }

  /** Updates the set of loggable inputs for both Shooter Motors */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  // /** Updates the PID values to what they are set to on the SmartDashboard */
  // public void updatePIDController() {
  //   FeederConstants.KP = feederkP.getDouble(0.0);
  //   FeederConstants.KP = feederkP.getDouble(0.0);
  //   FeederConstants.KP = feederkP.getDouble(0.0);
  //   feederPIDController.setPID(FeederConstants.KP, FeederConstants.KI, FeederConstants.KD);
  // }

  /** Updates the setpoint to what is typed on the SmartDashboard */
  // public void updateSetpoint() {
  //   setpointRPM = feederSetpointSetter.getDouble(0.0);
  //   feederPIDController.setSetpoint(setpointRPM);
  // }

  /**
   * Sets the voltage of the Feeder motor
   *
   * @param volts [-12 to 12]
   */
  public void setFeederVoltage(double volts) {
    io.setFeederVoltage(volts);
  }

  /**
   * Sets the speed of the Feeder motor based on a percent of its maximum speed
   *
   * @param percent [-1 to 1]
   */
  public void setFeederPercentSpeed(double percent) {
    io.setFeederPercentSpeed(percent);
  }

  /**
   * Sets the Feeder motor to brake mode
   *
   * @param enable
   */
  public void setFeederBrakeMode(boolean enable) {
    io.setFeederBrakeMode(enable);
  }

  public void setSetpoint(double setpoint) {
    feederPIDController.setSetpoint(setpoint);
  }

  public void disableFeeder() {
    feederPIDController.setSetpoint(0);
  }
}
