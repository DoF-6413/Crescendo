// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.OTBIntakeConstants;

/** Runs the real life OTBIntake with CANSpark Speed Controllers and NEO motor */
public class OTBIntakeIOSparkMax implements OTBIntakeIO {
  private final CANSparkMax OTBIntakeMotor;
  private final RelativeEncoder OTBIntakeEncoder;

  /** Creates the motor and encoder for the OTB Intake */
  public OTBIntakeIOSparkMax() {
    System.out.println("[Init] Creating UTBIntakeIO");
    OTBIntakeMotor = new CANSparkMax(OTBIntakeConstants.OTB_INTAKE_CANID, MotorType.kBrushless);
    OTBIntakeEncoder = OTBIntakeMotor.getEncoder();
  }

  /** Updates the values for the OTB Intake */
  public void updateInputs(OTBIntakeIOInputs inputs) {
    inputs.otbIntakeVelocityRPM =
        OTBIntakeEncoder.getVelocity()
            / OTBIntakeConstants
                .GEAR_RATIO; // Converts rotaions to Radians and then divides it by the gear ratio
    inputs.otbIntakeAppliedVolts =
        OTBIntakeMotor.getAppliedOutput()
            * OTBIntakeMotor.getBusVoltage(); // Applied voltage of the OTBIntake
    inputs.otbIntakeCurrentAmps =
        new double[] {OTBIntakeMotor.getOutputCurrent()}; // Amps used by intake
    inputs.otbIntakeTempCelsius = new double[] {OTBIntakeMotor.getMotorTemperature()}; // The tempature of the OTBIntake motor in Celsius
  }

  /**
   * Sets the voltage of the OTB Intake motor
   *
   * @param voltage [-12 to 12]
   */
  public void setOTBIntakeVoltage(double volts) {
    OTBIntakeMotor.setVoltage(volts);
  }

  /**
   * Sets the OTB Intake to a percent of its max speed
   *
   * @param percent [-1 to 1]
   */
  public void setOTBIntakePercentSpeed(double percent) {
    OTBIntakeMotor.set(percent);
  }
}
