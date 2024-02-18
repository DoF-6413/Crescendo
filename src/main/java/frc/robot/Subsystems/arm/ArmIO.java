// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double armTurnAppliedVolts = 0.0;
    public double armTurnPositionRad = 0.0;
    public double armTurnVelocityRadPerSec = 0.0;
    public double[] armTurnCurrentAmps = new double[] {};
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
   * @param volts
   */
  public default void setArmVoltage(double volts) {}
}
