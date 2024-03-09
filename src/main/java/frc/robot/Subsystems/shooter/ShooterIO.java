// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of All Shooter Modes */
public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    // All the Inputs for the Top Shooter Motor
    /** Velocity of the Top Shooter Motor in Rotations per Minute */
    public double topShooterMotorRPM = 0.0;
    /** Number of volts being sent to the Top Shooter Motor */
    public double topShooterAppliedVolts = 0.0;
    /** Number of amps used by the Top Shooter Motor */
    public double[] topShooterCurrentAmps = new double[] {};
    /** Tempature, in Celsius, of the Top Shooter Motor */
    public double[] topShooterTempCelsius = new double[] {};

    // All the Inputs for the Bottom Shooter Motor
    /** Velocity of the Bottom Shooter Motor in Rotations per Minute */
    public double bottomShooterMotorRPM = 0.0;
    /** Number of volts being sent to the Bottom Shooter Motor */
    public double bottomShooterAppliedVolts = 0.0;
    /** Number of amps used by the Bottom Shooter Motor */
    public double[] bottomShooterCurrentAmps = new double[] {};
    /** Tempature, in Celsius, of the Bottom Shooter Motor */
    public double[] bottomShooterTempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs for both Shooter Motors */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Sets the Brake Mode for the Shooter (Brake means motor holds position, Coast means easy to
   * move)
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public default void setBrakeMode(boolean enable) {}

  /**
   * Sets BOTH Shooter Motors at a percentage of its max speed.
   *
   * <p>A positve number spins the Top Shooter Motor CCW and the Bottom Shooter Motor CW and vice
   * versa for a negative number
   *
   * @param percent -1 to 1
   */
  public default void setBothShooterMotorPercentSpeed(double percent) {}

  /**
   * Sets BOTH Shooter Motors at the specified Voltage
   *
   * <p>A positve number spins the Top Shooter Motor CCW and the Bottom Shooter Motor CW and vice
   * versa for a negative number
   *
   * @param volts -12 to 12
   */
  public default void setBothShooterMotorsVoltage(double volts) {}

  /**
   * Sets the voltage of the Top Shooter Motor
   *
   * @param volts -12 to 12
   */
  public default void setTopShooterMotorVoltage(double volts) {}

  /**
   * Sets the voltage of the Bottom Shooter Motor
   *
   * @param volts -12 to 12
   */
  public default void setBottomShooterMotorVoltage(double volts) {}
}
