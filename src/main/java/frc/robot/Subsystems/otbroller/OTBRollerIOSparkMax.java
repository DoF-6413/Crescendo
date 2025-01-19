// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbroller;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Implementation of the OTB Roller IO for the real motor inputs and outputs */
public class OTBRollerIOSparkMax implements OTBRollerIO {
  private final SparkMax otbRollerMotor;
  private final SparkMaxConfig otbRollerConfig;
  private final RelativeEncoder otbRollerEncoder;

  public OTBRollerIOSparkMax() {
    System.out.println("[Init] Creating OTBRollerIOSparkMax");

    otbRollerMotor = new SparkMax(OTBRollerConstants.CAN_ID, MotorType.kBrushless);
    otbRollerEncoder = otbRollerMotor.getEncoder();
    otbRollerConfig = new SparkMaxConfig();

    otbRollerConfig.inverted(OTBRollerConstants.IS_INVERTED);
    otbRollerConfig.idleMode(IdleMode.kBrake);
    otbRollerConfig.smartCurrentLimit(OTBRollerConstants.CUR_LIM_A);

    otbRollerMotor.configure(otbRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
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
  public void setVoltage(double voltage) {
    otbRollerMotor.setVoltage(voltage);
  }

  @Override
  public void setPercentSpeed(double percent) {
    otbRollerMotor.set(percent);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    otbRollerConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    otbRollerMotor.configure(otbRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
