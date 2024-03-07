// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** Runs the real life OTBIntake with CANSpark Speed Controllers and NEO motor */
public class OTBIntakeIOSparkMax implements OTBIntakeIO {
  private final CANSparkMax otbIntakeMotor;
  private final RelativeEncoder otbIntakeEncoder;

  /** Creates the motor and encoder for the OTB Intake */
  public OTBIntakeIOSparkMax() {
    System.out.println("[Init] Creating utbIntakeIO");
    otbIntakeMotor = new CANSparkMax(OTBIntakeConstants.CAN_ID, MotorType.kBrushless);
    otbIntakeEncoder = otbIntakeMotor.getEncoder();
    otbIntakeMotor.setIdleMode(IdleMode.kBrake);
    otbIntakeMotor.setSmartCurrentLimit(OTBIntakeConstants.CURR_LIM_A);
    otbIntakeMotor.setInverted(OTBIntakeConstants.IS_INVERTED);
  }

  /** Updates the values for the OTB Intake */
  public void updateInputs(OTBIntakeIOInputs inputs) {
    inputs.otbIntakeVelocityRPM =
        otbIntakeEncoder.getVelocity()
            / OTBIntakeConstants
                .GEAR_RATIO; // Returns the RPM of the OTB Intake Rollers divided by the gear ratio
    // to obtain the speed of the OTB Intake Rollers
    inputs.otbIntakeAppliedVolts =
        otbIntakeMotor.getAppliedOutput() * otbIntakeMotor.getBusVoltage();
    inputs.otbIntakeCurrentAmps = new double[] {otbIntakeMotor.getOutputCurrent()};
    inputs.otbIntakeTempCelsius = new double[] {otbIntakeMotor.getMotorTemperature()};
  }

  @Override
  public void setOTBIntakeVoltage(double volts) {
    otbIntakeMotor.setVoltage(volts);
  }

  @Override
  public void setOTBIntakePercentSpeed(double percent) {
    otbIntakeMotor.set(percent);
  }

  @Override
  public void enableRollers(boolean enable) {
    if (enable) {
      setOTBIntakePercentSpeed(100);
    } else {
      setOTBIntakePercentSpeed(0);
    }
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (enable) {
      otbIntakeMotor.setIdleMode(IdleMode.kBrake);
    } else {
      otbIntakeMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}
