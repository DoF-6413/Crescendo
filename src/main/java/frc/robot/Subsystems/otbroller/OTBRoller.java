// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbroller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class OTBRoller extends SubsystemBase {
  private final OTBRollerIO io;
  private final OTBRollerIOInputsAutoLogged inputs;

  /** Creates a new OTBRoller. */
  public OTBRoller(OTBRollerIO io) {
    System.out.println("[Init] Creating OTB Roller");

    this.io = io;
    inputs = new OTBRollerIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("OTB/Roller", inputs);
  }

  /**
   * Sets OTB Roller Voltage
   *
   * @param volts -12 to 12
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Sets OTB Roller Percent Speed
   *
   * @param percent -1 to 1
   */
  public void setPercentSpeed(double percent) {
    io.setPercentSpeed(percent);
  }

  /**
   * Sets brake mode of the OTB Roller
   *
   * @param enable Enables brake mode if true, coast if false
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }
}
