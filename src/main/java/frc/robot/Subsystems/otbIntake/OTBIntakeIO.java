// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import org.littletonrobotics.junction.AutoLog;

/** These are all the input/output values that you can log for OTB Intake */
public interface OTBIntakeIO {

  @AutoLog
  public static class OTBIntakeIOInputs {
    public double otbIntakeVelocityRPM = 0.0;
    public double otbIntakeAppliedVolts = 0.0;
    public double[] otbIntakeCurrentAmps = new double[] {};
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
}
