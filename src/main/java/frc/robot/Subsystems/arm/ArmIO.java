// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/** ArmIO controls the Values of the Outputs User Recieve (Reffered to as Inputs because Inputs from Motors) */
public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {

    /** This returns the voltage the Actuator Recieves */
    public double armAppliedVolts = 0.0;
    /** Returns the position of the Actuator in Radians */
    public double armPositionRad = 0.0;
    /** Returns the velocity in Rad/s */
    public double armVelocityRadPerSec = 0.0;
    /** The Current Drawn from the Actuator in Amps */
    public double[] armCurrentAmps = new double[] {};
    /** The Temperature from the Actuator in Celsius */
    public double[] armTempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs for the Arm */
  public default void updateInputs(ArmIOInputs inputs) {}

  /**
   * Sets the Arm motor to a percent of its maximum speed
   * 
   * @param percent [-1 to 1]
   */
  public default void setArmPercentSpeed(double percent) {}


  /**
   * Sets the voltage of the Arm motor
   * 
   * @param volts [-12 to 12]
   */
  public default void setArmVoltage(double volts) {}

  /**Sets the Brake Mode for the Arm (Brake means motor holds position, Coast means easy to move) 
   * @param enable if enable, it sets brake mode, else it sets coast mode
  */
  public default void setBrakeMode(boolean enable){}
}
