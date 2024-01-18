// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of All Shooter Modes */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // All the Inputs for the Left Shooter Motor (Should be nearly identical to the Right Shooter
    // Motor)
    public double leftShooterMotorRPM = 0.0;
    public double leftShooterAppliedVolts = 0.0;
    public double[] leftShooterCurrentAmps = new double[] {};
    public double[] leftShooterTempCelcius = new double[] {};

    // All the Inputs for the Right Shooter Motor (Should be nearly identical to the Left Shooter
    // Motor)
    public double rightShooterMotorRPM = 0.0;
    public double rightShooterAppliedVolts = 0.0;
    public double[] rightShooterCurrentAmps = new double[] {};
    public double[] rightShooterTempCelcius = new double[] {};
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets ALL Shooter Motors at the specified Voltage */
  public default void setShooterMotorsVoltage(double volts) {}

  /** Break Mode for BOTH Shooter Motors */
  public default void setShooterBreakMode(boolean enable) {}

  public default void setShooterMotorPercentSpeed(double percent) {}
}
