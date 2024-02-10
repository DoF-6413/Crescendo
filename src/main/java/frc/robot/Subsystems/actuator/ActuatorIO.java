// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import org.littletonrobotics.junction.AutoLog;

public interface ActuatorIO {
  /** Creates a new ActuatorIO. */
  @AutoLog
  public static class ActuatorIOInputs {

    public double turnAppliedVolts = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnCurrentAmps = 0.0;
  }

  public default void updateInputs(ActuatorIOInputs inputs) {
    // This method will be called once per scheduler run
  }

  
  public default void setActuatorVoltage(double volts) {}


public void setActuatorPosition(double calculate);


public void setActuatorSpeed(double voltage);
}
