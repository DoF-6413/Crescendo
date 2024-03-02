// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// 2 motors
// neo

package frc.robot.Subsystems.wrist;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristIOSparkMax implements WristIO {

  private final CANSparkMax wristMotor;
  private final RelativeEncoder wristRelativeEncoder;
  private final SparkAbsoluteEncoder wristAbsoluteEncoder;

  // private final RelativeEncoder wristEncoder;

  public WristIOSparkMax() {
    /** creates a new wrist motor and encoder */
    wristMotor = new CANSparkMax(WristConstants.CAN_ID, MotorType.kBrushless);
    wristRelativeEncoder = wristMotor.getEncoder();
    // wristAbsoluteEncoder = new SparkMaxAlternateEncoder(wristMotor, , 8192);
    wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristAbsoluteEncoder.setInverted(true);
    wristAbsoluteEncoder.setZeroOffset(0);

    /** sets default to brake mode, which locks the motor position */
    wristMotor.setIdleMode(IdleMode.kBrake);

    /** sets current limit */
    wristMotor.setSmartCurrentLimit(WristConstants.CUR_LIM_A);
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
    inputs.wristAbsolutePositionRad = (wristAbsoluteEncoder.getPosition() * 2 * Math.PI) - 2.8;
    inputs.wristAbsolutePositionDeg =
        (wristAbsoluteEncoder.getPosition() * 360) - Units.radiansToDegrees(2.8);
    inputs.wristVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wristRelativeEncoder.getVelocity());
    inputs.wristTempCelsius = new double[] {wristMotor.getMotorTemperature()};
    inputs.wristCurrentAmps = new double[] {wristMotor.getOutputCurrent()};
    SmartDashboard.putNumber("Absolute Encoder", wristAbsoluteEncoder.getPosition());
  }

  @Override
  public void setWristPercentSpeed(double percent) {
    wristMotor.set(percent);
  }

  @Override
  public void setWristVoltage(double volts) {
    wristMotor.setVoltage(volts);
  }

  @Override
  public void setWristBrakeMode(boolean enable) {
    if (enable) {
      wristMotor.setIdleMode(IdleMode.kBrake);
    } else {
      wristMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}
