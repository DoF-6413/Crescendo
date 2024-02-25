// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of Each Individual Module */
public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    /** This returns the voltage the Propulsion Motor Recieves */
    public double driveAppliedVolts = 0.0;
    /** Returns the position of the Propulsion Motor by how many radians it has rotated */
    public double drivePositionRad = 0.0;
    /**
     * Returns the velocity of the Propulsion Motor by how many radians per second it has rotated
     */
    public double driveVelocityRadPerSec = 0.0;
    /**
     * Returns the absoltute value of the velocity of the Propulsion Motor by how many radians per
     * second it has rotated (Used to find Displacement)
     */
    public double driveVelocityRadPerSecAbs = Math.abs(0.0);
    /** The Current Drawn from the Propulsion Motor in Amps */
    public double[] driveCurrentAmps = new double[] {};
    /** The tempature of the Propulsion Motor in Celsius */
    public double[] driveTempCelsius = new double[] {};

    /** This returns the voltage the Steer Motor Recieves */
    public double turnAppliedVolts = 0.0;
    /**
     * Returns the position of the absoltute encoder in Radians (Used to make sure wheel zero doesnt
     * change on enable
     */
    public double turnAbsolutePositionRad = 0.0;
    /** Returns the position of the Steer Motor by how many radians it has rotated */
    public double turnPositionRad = 0.0;
    /** Returns the velocity of the Steer Motor by how many radians per second it has rotated */
    public double turnVelocityRadPerSec = 0.0;
    /** The Current Drawn from the Steer Motor in Amps */
    public double[] turnCurrentAmps = new double[] {};
    /** The tempature of the Steer Motor in Celsius */
    public double[] turnTempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}

  /** Determines whether Krakens are being Used for the Propulsion or Not */
  public default Optional<Boolean> isL3() {
    return null;
  }
}
