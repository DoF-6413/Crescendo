// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;


public class ActuatorIO extends SubsystemBase {
  /** Creates a new ActuatorIO. */
    @AutoLog
    public static class ActuatorIOInputs{
    

    public double turnAppliedVolts = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnCurrentAmps = 0.0;
    public double WristTempCelcius = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
