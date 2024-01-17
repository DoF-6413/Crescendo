// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of All Shooter Modes */
public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs{

    }
    /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /**Sets Left Shooter Motor at the specified Voltage */
  public default void setLeftShooterMotorVoltage(double volts){}

  /**Sets Right Shooter Motor at the specified Voltage */
  public default void setRigtShooterMotorVoltage(double volts){}

  /**Break Mode for BOTH Shooter Motors */
  public default void setShooterBreakMode(boolean enable){}
}
