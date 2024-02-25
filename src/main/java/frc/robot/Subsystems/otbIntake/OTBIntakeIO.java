// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import org.littletonrobotics.junction.AutoLog;

/** These are all the input/output values that you can log for OTB Intake */
public interface OTBIntakeIO {

  @AutoLog
  public static class OTBIntakeIOInputs {
    /** Velocity of the OTB Intake Rollers in Rotations per Minute */
    public double otbIntakeVelocityRPM = 0.0;
    /** Number of volts being sent to the OTB Intake motor */
    public double otbIntakeAppliedVolts = 0.0;
    /** Number of Amps being used by the OTB Intake motor */
    public double[] otbIntakeCurrentAmps = new double[] {};
    /** Tempature of the OTB Intake motor */
    public double[] otbIntakeTempCelsius = new double[] {};
  }

  /** Updates inputs for the OTB Intake */
  public default void updateInputs(OTBIntakeIOInputs inputs) {}

  /**
   * Sets the voltage for the OTB Intake
   *
   * @param volts [-12 to 12]
   */
  public default void setOTBIntakeVoltage(double volts) {}

  /**
   * Sets the speed for the OTB Intake
   *
   * @param percent [-1 to 1]
   */
  public default void setOTBIntakePercentSpeed(double percent) {}

  /**
   * Sets the Brake Mode for the OTB Intake
   *
   * <p>Brake means motor holds position, Coast means easy to move
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public default void setBrakeMode(boolean enable) {}

  public default void enableRollers(boolean enable) {}
}
