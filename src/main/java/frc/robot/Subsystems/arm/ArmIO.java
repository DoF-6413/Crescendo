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
    public double armTurnCurrentAmps = 0.0;
    public double armTempCelcius = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmMotorSpeed(double Speed) {
  }
}
