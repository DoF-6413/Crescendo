// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import org.littletonrobotics.junction.AutoLog;

/**
 * ActuatorIO controls the Values of the Outputs User Recieve (Reffered to as Inputs because Inputs
 * from Motors)
 */
public interface ActuatorIO {

  @AutoLog
  public static class ActuatorIOInputs {

    /** This returns the voltage the Actuator Recieves */
    public double actuatorAppliedVolts = 0.0;
    /** Returns the position of the Actuator in Radians */
    public double actuatorPositionRad = 0.0;
    /** Returns the position of the Actuator in Degrees */
    public double actuatorPositionDeg = 0.0;
    /** Returns the velocity of the Actuator in Rad/s */
    public double actuatorVelocityRadPerSec = 0.0;
    /** The Current Drawn from the Actuator in Amps */
    public double actuatorCurrentAmps = 0;
    /** The Temperature from the Actuator in Celsius */
    public double actuatorTempCelsius = 0;
  }

  /** Updates inputs for the Actuator */
  public default void updateInputs(ActuatorIOInputs inputs) {}

  /**
   * Sets the voltage of the Actuator motor
   *
   * @param volts -12 to 12
   */
  public default void setMotorVoltage(double volts) {}

  /**
   * Sets the Actuator motor to a percentage of its max speed
   *
   * @param percent -1 to 1
   */
  public default void setPercentSpeed(double percent) {}

  /**
   * Sets the Brake Mode for the Actuator (Brake means motor holds position, Coast means easy to
   * move)
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public default void setBrakeMode(boolean enable) {}

  /**
   * Sets the smart current limiting of the Actuator using the SPARK MAX speed contollers
   *
   * @param current Amps
   */
  public default void setCurrentLimit(int current) {}

  /** Resets the current position of the Actuator to be the new zero position */
  public default void zeroPosition() {}
}
