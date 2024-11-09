// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbroller;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the OTB Roller */
public interface OTBRollerIO {

  @AutoLog
  public static class OTBRollerIOInputs {
    /** The velocity of the OTB Roller in Rotations per Minute */
    public double otbRollerRPM = 0.0;
    /** Number of volts being sent to the OTB Roller motor */
    public double otbRollerAppliedVolts = 0.0;
    /** Number of amps being used by the OTB Roller motor */
    public double otbRollerCurrentAmps = 0;
    /** Tempature, in Celsius, of the OTB Roller motor */
    public double otbRollerTempCelsius = 0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(OTBRollerIOInputs inputs) {}

  /**
   * Sets OTB Roller Voltage
   *
   * @param volts -12 to 12
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets OTB Roller Percent Speed
   *
   * @param percent -1 to 1
   */
  public default void setPercentSpeed(double percent) {}

  /**
   * Sets brake mode of the OTB Roller
   *
   * @param enable Enables brake mode if true, coast if false
   */
  public default void setBrakeMode(boolean enable) {}
}
