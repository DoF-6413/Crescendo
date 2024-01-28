// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;


public class ActuatorIO {
  
  @AutoLog
  public static class ActuatorIOInputs{
  
    public double turnAppliedVolts = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnCurrentAmps = 0.0;
    public double WristTempCelcius = 0.0;
  }

 
  public void updateInputs(ActuatorIOInputs inputs) {}
}
