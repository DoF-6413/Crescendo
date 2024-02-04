// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of All Shooter Modes */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // All the Inputs for the Top Shooter Motor (Should be nearly identical to the Bottom Shooter
    // Motor)
    public double topShooterMotorRPM = 0.0;
    public double topShooterAppliedVolts = 0.0;
    public double[] topShooterCurrentAmps = new double[] {};
    public double[] topShooterTempCelcius = new double[] {};

    // All the Inputs for the Bottom Shooter Motor (Should be nearly identical to the Top Shooter
    // Motor)
    public double bottomShooterMotorRPM = 0.0;
    public double bottomShooterAppliedVolts = 0.0;
    public double[] bottomShooterCurrentAmps = new double[] {};
    public double[] bottomShooterTempCelcius = new double[] {};
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets ALL Shooter Motors at the specified Voltage */
  public default void setBothShooterMotorsVoltage(double volts) {}

  /** Break Mode for BOTH Shooter Motors */
  public default void setShooterBreakMode(boolean enable) {}

  /**
   * Sets ALL Shooter Motors at a percentage of its max speed. A positve number spins the Top
   * Shooter Motor CCW and the Bottom Shooter Motor CW and vice versa for a negative number
   *
   * @param percent -1 to 1
   */
  public default void setBothShooterMotorPercentSpeed(double percent) {}

  /** Sets the voltage of the Top Shooter Motor */
  public default void setTopShooterMotorVoltage(double volts) {}

  /** Sets the voltage of the Bottom Shooter Motor */
  public default void setBottomShooterMotorVoltage(double volts) {}
}
