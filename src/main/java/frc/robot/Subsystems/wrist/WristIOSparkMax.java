// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// 2 motors
// neo

package frc.robot.Subsystems.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

public class WristIOSparkMax implements WristIO {

  private final SparkMax wristMotor;
  private final RelativeEncoder wristRelativeEncoder;
  private final SparkAbsoluteEncoder wristAbsoluteEncoder;
  private final SparkMaxConfig wristConfig;


  public WristIOSparkMax() {
    /** Creates a new Wrist motor and encoder */
    wristMotor = new SparkMax(WristConstants.CAN_ID, MotorType.kBrushless);
    wristRelativeEncoder = wristMotor.getEncoder();
    wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder();
    wristConfig = new SparkMaxConfig();

    wristConfig.absoluteEncoder.inverted(WristConstants.IS_INVERTED);
    wristConfig.absoluteEncoder.zeroOffset(0.2);

    /** sets default to brake mode, which locks the motor position */
    wristConfig.idleMode(IdleMode.kBrake);

    /** sets current limit */
    wristConfig.smartCurrentLimit(WristConstants.CUR_LIM_A);

    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAppliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.wristRelativePositionRad =
        Units.rotationsToRadians(wristRelativeEncoder.getPosition()) / WristConstants.GEAR_RATIO;
    inputs.wristRelativePositionDeg =
        Units.rotationsToDegrees(wristRelativeEncoder.getPosition()) / WristConstants.GEAR_RATIO;
    // The absolute encoder, or a dut cycle encoder, rotates where a full rotation is equal to 1. If
    // 1 rotation is equal to 2pi or 360 degrees, multiply by appropriate to get value
    inputs.wristAbsolutePositionRad =
        (wristAbsoluteEncoder.getPosition() * 2 * Math.PI + WristConstants.ABS_ENCODER_OFFSET_RADS);
    inputs.wristAbsolutePositionDeg =
        (wristAbsoluteEncoder.getPosition() * 360)
            + Units.radiansToDegrees(WristConstants.ABS_ENCODER_OFFSET_RADS);
    inputs.wristVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wristRelativeEncoder.getVelocity());
    inputs.wristTempCelsius = wristMotor.getMotorTemperature();
    inputs.wristCurrentAmps = wristMotor.getOutputCurrent();
  }

  @Override
  public void setPercentSpeed(double percent) {
    wristMotor.set(percent);
  }

  @Override
  public void setVoltage(double volts) {
    wristMotor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (enable) {
      wristConfig.idleMode(IdleMode.kBrake);
    } else {
      wristConfig.idleMode(IdleMode.kCoast);
    }
  }
}
