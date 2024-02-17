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
    public double actuatorVelocityRadPerSec = 0.0;
    public double actuatorCurrentAmps = 0.0;
  }

  public default void updateInputs(ActuatorIOInputs inputs) {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the voltage for the Actuator
   *
   * @param volts [-12 to 12]
   */
  public default void setActuatorVoltage(double volts) {}

  /**
   * Sets the speed for the Actuator
   *
   * @param percent [-1 to 1]
   */
  public default void setActuatorPercentSpeed(double percent) {}
}
