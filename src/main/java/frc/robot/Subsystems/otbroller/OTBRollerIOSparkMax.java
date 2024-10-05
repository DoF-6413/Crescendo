// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbroller;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** Implementation of the OTB Roller IO for the real motor inputs and outputs */
public class OTBRollerIOSparkMax implements OTBRollerIO {
  private final CANSparkMax otbRollerMotor;
  private final RelativeEncoder otbRollerEncoder;

  public OTBRollerIOSparkMax() {
    System.out.println("[Init] Creating OTBRollerIOSparkMax");

    otbRollerMotor = new CANSparkMax(OTBRollerConstants.CAN_ID, MotorType.kBrushless);
    otbRollerEncoder = otbRollerMotor.getEncoder();

    otbRollerMotor.setInverted(OTBRollerConstants.IS_INVERTED);
    otbRollerMotor.setIdleMode(IdleMode.kBrake);
    otbRollerMotor.setSmartCurrentLimit(OTBRollerConstants.CUR_LIM_A);

    otbRollerMotor.burnFlash();
  }

  @Override
  public void updateInputs(OTBRollerIOInputs inputs) {
    inputs.otbRollerAppliedVolts =
        otbRollerMotor.getAppliedOutput() * otbRollerMotor.getBusVoltage();
    inputs.otbRollerCurrentAmps = otbRollerMotor.getOutputCurrent();
    inputs.otbRollerRPM = otbRollerEncoder.getVelocity() / OTBRollerConstants.GEAR_RATIO;
    inputs.otbRollerTempCelsius = otbRollerMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    otbRollerMotor.setVoltage(volts);
  }

  @Override
  public void setPercentSpeed(double percent) {
    otbRollerMotor.set(percent);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    otbRollerMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
