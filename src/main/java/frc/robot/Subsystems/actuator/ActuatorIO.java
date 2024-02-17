// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import org.littletonrobotics.junction.AutoLog;

public interface ActuatorIO {
  /** Creates a new ActuatorIO. */
  @AutoLog
  public static class ActuatorIOInputs {

    public double actuatorAppliedVolts = 0.0;
    public double actuatorPositionRad = 0.0;
    public double actuatorPositionM = 0.0;
    public double actuatorVelocityRadPerSec = 0.0;
    public double[] actuatorCurrentAmps = new double[] {};
    public double[] actuatorTempCelsius = new double[] {};
  }

  /** Updates inputs for the Actuator */
  public default void updateInputs(ActuatorIOInputs inputs) {
  }

  /**
   * Sets the voltage of the Actuator motor
   * 
   * @param volts [-12 to 12]
   */
  public default void setActuatorVoltage(double volts) {}

  /**
   * Sets the Actuator motor to a percentage of its max speed
   * 
   * @param percent [-1 to 1]
   */
  public default void setActuatorPercentSpeed(double percent) {}
}
